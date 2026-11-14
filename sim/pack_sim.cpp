// Pack simulation driver.
//
//   pack_sim [scenario.scn] [--seed N] [--quiet] [--trace out.json [--trace-period S]]
//   pack_sim [scenario.scn] --mc N [--threads N] [--mc-json out.json]
//
// A single run prints the timeline, a per-drone summary and the scenario's
// checks, and exits non-zero if any check fails. --mc runs N trials with
// different seeds (and the scenario's position and timing jitter) and prints
// the distributions of time to alert, position error and false alerts.
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include "sim.hpp"

namespace {

using sim::TrialResult;

void usage() {
  std::fprintf(stderr,
               "usage: pack_sim [scenario.scn] [--seed N] [--quiet] [--trace out.json] [--trace-period S]\n"
               "       pack_sim [scenario.scn] --mc N [--threads N] [--mc-json out.json]\n");
}

struct Dist {
  std::vector<double> v;
  void add(double x) { v.push_back(x); }
  double pct(double p) const {
    if (v.empty()) return NAN;
    std::vector<double> s = v;
    std::sort(s.begin(), s.end());
    const double pos = p / 100.0 * static_cast<double>(s.size() - 1);
    const auto lo = static_cast<std::size_t>(std::floor(pos));
    const std::size_t hi = std::min(lo + 1, s.size() - 1);
    return s[lo] + (s[hi] - s[lo]) * (pos - static_cast<double>(lo));
  }
  double mean() const {
    if (v.empty()) return NAN;
    double t = 0;
    for (const double x : v) t += x;
    return t / static_cast<double>(v.size());
  }
  double max() const { return v.empty() ? NAN : *std::max_element(v.begin(), v.end()); }
};

void printRun(const sim::Scenario& sc, const TrialResult& r) {
  std::printf("\n================ summary ================\n");
  std::printf("%-5s %-15s %-7s %7s %7s %5s %5s %7s %7s %7s %9s\n", "drone", "role", "tasked", "tx", "rx", "bad", "auth",
              "tasks", "claims", "lost", "tx kbit/s");
  for (const auto& d : r.drones) {
    const auto& s = d.stats;
    std::printf("D%-4d %-15s %-7s %7u %7u %5u %5u %7u %7u %7u %9.2f%s\n", d.id, d.active ? "ACTIVE_MISSION" : "PACK_ROUTING",
                d.tasked ? "yes" : "no", s.tx_packets, s.rx_packets, s.rx_rejected + s.rx_stale + s.rx_replayed,
                s.rx_auth_failed, s.tasks_issued, s.claims_sent, s.peers_lost, d.tx_kbps, d.alive ? "" : "  (lost)");
  }
  std::printf("\nmesh traffic: %llu datagrams, %llu bytes, %llu deliveries lost on the radio (bursty)\n",
              static_cast<unsigned long long>(r.datagrams), static_cast<unsigned long long>(r.bytes),
              static_cast<unsigned long long>(r.datagrams_lost));
  std::printf("offered load (sealed frames, all types): mean %.2f kbit/s per drone, peak %.2f kbit/s per drone in any 1 s\n",
              r.mean_kbps_per_drone, r.peak_kbps_per_drone);
  std::printf("alerts at the GMCS: %zu (false: %d advisory, %d priority; duplicates %d)\n", r.alerts.size(),
              r.false_advisories, r.false_priorities, r.duplicate_alerts);
  std::printf("GMCS records after de-duplication: %d (false %d, duplicate %d, missed acknowledgement deadlines %d)\n",
              r.gmcs.records, r.gmcs.false_records, r.gmcs.duplicate_records, r.gmcs.escalations);
  for (std::size_t k = 0; k < sc.emitters.size(); ++k) {
    const auto& e = sc.emitters[k];
    if (!e.alertable) continue;
    const auto& o = r.emitters[k];
    std::printf("  %-14s first alert ", e.label.c_str());
    if (o.t_advisory >= 0) {
      std::printf("%.1f s after switch-on", o.t_advisory);
    } else {
      std::printf("never");
    }
    std::printf("; PRIORITY ");
    if (o.t_priority >= 0) {
      std::printf("%.1f s, error %.0f m\n", o.t_priority, o.err_priority_m);
    } else {
      std::printf("never\n");
    }
  }
  std::printf("\n");
  for (const auto& c : r.checks) std::printf("  [%s] %s\n", c.ok ? "PASS" : "FAIL", c.text.c_str());
  std::printf("\n%s\n", r.passed() ? "scenario result: PASS" : "scenario result: FAIL");
}

void jsonDist(FILE* f, const char* name, const Dist& d, bool last = false) {
  std::fprintf(f, "\"%s\":{\"n\":%zu", name, d.v.size());
  if (!d.v.empty()) {
    std::fprintf(f, ",\"mean\":%.3f,\"p5\":%.3f,\"p25\":%.3f,\"p50\":%.3f,\"p75\":%.3f,\"p95\":%.3f,\"max\":%.3f,\"samples\":[",
                 d.mean(), d.pct(5), d.pct(25), d.pct(50), d.pct(75), d.pct(95), d.max());
    for (std::size_t i = 0; i < d.v.size(); ++i) std::fprintf(f, "%s%.3f", i ? "," : "", d.v[i]);
    std::fprintf(f, "]");
  }
  std::fprintf(f, "}%s", last ? "" : ",");
}

void printDist(const char* name, const Dist& d, const char* unit, std::size_t of) {
  if (d.v.empty()) {
    std::printf("  %-34s none in %zu trials\n", name, of);
    return;
  }
  std::printf("  %-34s %5.1f%%  p5 %7.1f  p50 %7.1f  p95 %7.1f  max %7.1f %s\n", name,
              100.0 * static_cast<double>(d.v.size()) / static_cast<double>(of), d.pct(5), d.pct(50), d.pct(95), d.max(),
              unit);
}

int monteCarlo(const sim::Scenario& sc, int trials, unsigned threads, const std::string& json_path) {
  std::vector<TrialResult> results(static_cast<std::size_t>(trials));
  std::atomic<int> next{0};
  const auto worker = [&]() {
    for (int i = next++; i < trials; i = next++) {
      sim::RunOptions opt;
      opt.seed = sc.seed + static_cast<std::uint32_t>(i) * 7919U;
      opt.mc_jitter = true;
      results[static_cast<std::size_t>(i)] = sim::runScenario(sc, opt);
    }
  };
  std::vector<std::thread> pool;
  for (unsigned t = 0; t < std::max(1U, threads); ++t) pool.emplace_back(worker);
  for (auto& th : pool) th.join();

  const std::size_t n = results.size();
  std::printf("Monte Carlo: %s, %d trials (%.0f s each, emitters jittered by up to %.1f km and %.0f s)\n\n", sc.name.c_str(),
              trials, sc.duration_s, sc.mc_jitter_m / 1000.0, sc.mc_jitter_on_s);
  std::printf("  %-34s %6s  %10s  %10s  %10s  %10s\n", "", "rate", "", "", "", "");

  std::vector<Dist> t_adv(sc.emitters.size()), t_pri(sc.emitters.size()), err_pri(sc.emitters.size());
  Dist false_adv, false_pri, dup, load_mean, load_peak, fcs, recs, false_recs, dup_recs;
  std::array<Dist, 3> transport, detection;
  double hours = 0;
  int false_adv_total = 0, false_pri_total = 0, false_clutter = 0, false_multipath = 0;
  std::vector<int> check_pass(sc.expects.size(), 0);
  for (const auto& r : results) {
    for (std::size_t k = 0; k < sc.emitters.size(); ++k) {
      const auto& o = r.emitters[k];
      if (o.t_advisory >= 0) t_adv[k].add(o.t_advisory);
      if (o.t_priority >= 0) {
        t_pri[k].add(o.t_priority);
        err_pri[k].add(o.err_priority_m);
      }
    }
    false_adv.add(r.false_advisories);
    false_pri.add(r.false_priorities);
    false_adv_total += r.false_advisories;
    false_pri_total += r.false_priorities;
    false_clutter += r.false_from_clutter;
    false_multipath += r.false_from_multipath;
    dup.add(r.duplicate_alerts);
    fcs.add(r.max_fcs_requests);
    recs.add(r.gmcs.records);
    false_recs.add(r.gmcs.false_records);
    dup_recs.add(r.gmcs.duplicate_records);
    for (std::size_t l = 0; l < 3; ++l) {
      for (const double x : r.gmcs.transport_ms[l]) transport[l].add(x);
      for (const double x : r.gmcs.detection_s[l]) detection[l].add(x);
    }
    load_mean.add(r.mean_kbps_per_drone);
    load_peak.add(r.peak_kbps_per_drone);
    hours += r.hours_flown;
    for (std::size_t c = 0; c < r.checks.size() && c < check_pass.size(); ++c) check_pass[c] += r.checks[c].ok ? 1 : 0;
  }
  for (std::size_t k = 0; k < sc.emitters.size(); ++k) {
    if (!sc.emitters[k].alertable) continue;
    std::printf("  %s\n", sc.emitters[k].label.c_str());
    printDist("    time to first alert", t_adv[k], "s", n);
    printDist("    time to PRIORITY", t_pri[k], "s", n);
    printDist("    PRIORITY position error", err_pri[k], "m", n);
  }
  std::printf("\n  false alerts (no real, alertable source within max(2 km, 3 CEP) of the alert):\n");
  std::printf("    ADVISORY  %d in %.0f drone-hours = %.2f per drone-hour; trials with any: %.1f%%; per trial p95 %.0f max %.0f\n",
              false_adv_total, hours, false_adv_total / std::max(hours, 1e-9),
              100.0 * static_cast<double>(std::count_if(false_adv.v.begin(), false_adv.v.end(), [](double x) { return x > 0; })) /
                  static_cast<double>(n),
              false_adv.pct(95), false_adv.max());
  std::printf("    PRIORITY  %d in %.0f drone-hours = %.3f per drone-hour; trials with any: %.1f%%\n", false_pri_total, hours,
              false_pri_total / std::max(hours, 1e-9),
              100.0 * static_cast<double>(std::count_if(false_pri.v.begin(), false_pri.v.end(), [](double x) { return x > 0; })) /
                  static_cast<double>(n));
  std::printf("    causes: %d from clutter, %d a real source mislocalised by multipath\n", false_clutter, false_multipath);
  std::printf("  duplicate alerts (same source, same level) per trial: mean %.2f, p95 %.0f, max %.0f\n", dup.mean(),
              dup.pct(95), dup.max());
  std::printf("\n  at the GMCS, after the alert consumer's de-duplication:\n");
  std::printf("    records per trial: mean %.2f; false records %.2f per drone-hour; records beyond the first for one source: mean %.2f, max %.0f\n",
              recs.mean(), false_recs.mean() * static_cast<double>(n) / std::max(hours, 1e-9), dup_recs.mean(), dup_recs.max());
  static const char* kLinks[3] = {"LOS", "LEO", "GEO"};
  for (std::size_t l = 0; l < 3; ++l) {
    if (transport[l].v.empty()) continue;
    std::printf("    transport latency %s: p50 %.0f ms, p95 %.0f ms (%zu alerts)\n", kLinks[l], transport[l].pct(50),
                transport[l].pct(95), transport[l].v.size());
  }
  if (!detection[1].v.empty()) {
    std::printf("    first detection -> ADVISORY at the GMCS: p50 %.1f s, p95 %.1f s\n", detection[1].pct(50), detection[1].pct(95));
  }
  if (!detection[2].v.empty()) {
    std::printf("    first detection -> PRIORITY at the GMCS: p50 %.1f s, p95 %.1f s\n", detection[2].pct(50), detection[2].pct(95));
  }
  std::printf("\n");
  std::printf("  FCS requests by the busiest drone per trial: p50 %.0f, p95 %.0f, max %.0f\n", fcs.pct(50), fcs.pct(95),
              fcs.max());
  std::printf("  mesh load per drone: mean %.2f kbit/s (p95 over trials %.2f); peak 1 s window p95 %.2f kbit/s\n",
              load_mean.mean(), load_mean.pct(95), load_peak.pct(95));
  std::printf("\n  scenario checks, pass rate over trials:\n");
  for (std::size_t c = 0; c < sc.expects.size(); ++c) {
    std::printf("    %5.1f%%  %s\n", 100.0 * check_pass[c] / static_cast<double>(n), sc.expects[c].text.c_str());
  }

  if (!json_path.empty()) {
    FILE* f = std::fopen(json_path.c_str(), "w");
    if (f == nullptr) {
      std::fprintf(stderr, "cannot write %s\n", json_path.c_str());
      return 1;
    }
    std::fprintf(f, "{\"scenario\":\"%s\",\"trials\":%d,\"duration_s\":%.1f,\"jitter_m\":%.0f,\"jitter_on_s\":%.0f,\"drone_hours\":%.2f,",
                 sc.name.c_str(), trials, sc.duration_s, sc.mc_jitter_m, sc.mc_jitter_on_s, hours);
    std::fprintf(f, "\"emitters\":[");
    bool first = true;
    for (std::size_t k = 0; k < sc.emitters.size(); ++k) {
      if (!sc.emitters[k].alertable) continue;
      std::fprintf(f, "%s{\"label\":\"%s\",", first ? "" : ",", sc.emitters[k].label.c_str());
      first = false;
      jsonDist(f, "time_to_first_alert_s", t_adv[k]);
      jsonDist(f, "time_to_priority_s", t_pri[k]);
      jsonDist(f, "priority_error_m", err_pri[k], true);
      std::fprintf(f, "}");
    }
    std::fprintf(f, "],\"false_alerts\":{\"advisory_total\":%d,\"priority_total\":%d,\"from_clutter\":%d,\"from_multipath\":%d,",
                 false_adv_total, false_pri_total, false_clutter, false_multipath);
    jsonDist(f, "advisory_per_trial", false_adv);
    jsonDist(f, "priority_per_trial", false_pri, true);
    std::fprintf(f, "},");
    jsonDist(f, "duplicates_per_trial", dup);
    jsonDist(f, "max_fcs_requests_per_trial", fcs);
    jsonDist(f, "gmcs_records_per_trial", recs);
    jsonDist(f, "gmcs_false_records_per_trial", false_recs);
    jsonDist(f, "gmcs_duplicate_records_per_trial", dup_recs);
    jsonDist(f, "transport_ms_los", transport[0]);
    jsonDist(f, "transport_ms_leo", transport[1]);
    jsonDist(f, "transport_ms_geo", transport[2]);
    jsonDist(f, "detection_to_advisory_s", detection[1]);
    jsonDist(f, "detection_to_priority_s", detection[2]);
    jsonDist(f, "mean_kbps_per_drone", load_mean);
    jsonDist(f, "peak_kbps_per_drone", load_peak);
    std::fprintf(f, "\"checks\":[");
    for (std::size_t c = 0; c < sc.expects.size(); ++c) {
      std::fprintf(f, "%s{\"text\":\"%s\",\"pass_rate\":%.4f}", c ? "," : "", sc.expects[c].text.c_str(),
                   check_pass[c] / static_cast<double>(n));
    }
    std::fprintf(f, "]}\n");
    std::fclose(f);
    std::printf("\nwrote %s\n", json_path.c_str());
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string path = "sim/scenarios/storyline6.scn";
  sim::RunOptions opt;
  bool seed_set = false;
  int mc = -1;
  unsigned threads = std::max(1U, std::thread::hardware_concurrency());
  std::string mc_json;
  opt.verbose = true;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const auto next = [&]() -> const char* {
      if (i + 1 >= argc) {
        usage();
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--seed") {
      opt.seed = static_cast<std::uint32_t>(std::strtoul(next(), nullptr, 0));
      seed_set = true;
    } else if (a == "--quiet") {
      opt.verbose = false;
    } else if (a == "--trace") {
      opt.trace_path = next();
    } else if (a == "--trace-period") {
      opt.trace_period_s = std::strtod(next(), nullptr);
    } else if (a == "--mc") {
      mc = std::atoi(next());
    } else if (a == "--threads") {
      threads = static_cast<unsigned>(std::atoi(next()));
    } else if (a == "--mc-json") {
      mc_json = next();
    } else if (!a.empty() && a[0] == '-') {
      usage();
      return 2;
    } else {
      path = a;
    }
  }
  sim::Scenario sc;
  std::string error;
  if (!sim::loadScenario(path, sc, error)) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 2;
  }
  if (mc >= 0) return monteCarlo(sc, mc == 0 ? sc.mc_trials : mc, threads, mc_json);

  if (!seed_set) opt.seed = sc.seed;
  std::printf("Model 8270 pack mesh - scenario %s (%zu drones, %.0f s, seed %u)\n\n", sc.name.c_str(), sc.drones.size(),
              sc.duration_s, opt.seed);
  const TrialResult r = sim::runScenario(sc, opt);
  printRun(sc, r);
  if (!opt.trace_path.empty()) std::printf("trace written to %s\n", opt.trace_path.c_str());
  return r.passed() ? 0 : 1;
}
