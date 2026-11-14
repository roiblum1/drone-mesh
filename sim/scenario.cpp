// Scenario file parser. One statement per line, '#' starts a comment.
// Distances in km unless a key says otherwise; see sim/scenarios/*.scn.
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

#include "amc8270/mesh/validation.hpp"
#include "sim.hpp"

namespace sim {

namespace {

struct Ctx {
  const std::string& path;
  int line = 0;
  std::string& error;
  bool fail(const std::string& msg) {
    error = path + ":" + std::to_string(line) + ": " + msg;
    return false;
  }
};

bool toDouble(const std::string& s, double& out) {
  char* end = nullptr;
  out = std::strtod(s.c_str(), &end);
  return end != s.c_str() && *end == '\0' && std::isfinite(out);
}

bool toInt(const std::string& s, long& out) {
  char* end = nullptr;
  out = std::strtol(s.c_str(), &end, 0);
  return end != s.c_str() && *end == '\0';
}

bool parseClass(const std::string& s, mesh::EventClass& out) {
  static const std::map<std::string, mesh::EventClass> k = {
      {"UNKNOWN", mesh::EventClass::UNKNOWN},       {"RF_EMITTER", mesh::EventClass::RF_EMITTER},
      {"ANOMALY", mesh::EventClass::ANOMALY},       {"THERMAL_HAZARD", mesh::EventClass::THERMAL_HAZARD},
      {"DISTRESS_BEACON", mesh::EventClass::DISTRESS_BEACON}};
  const auto it = k.find(s);
  if (it == k.end()) return false;
  out = it->second;
  return true;
}

// "RF+EO+IR+AN" / "LOS+LEO+GEO" / "NONE"
bool parseMask(const std::string& s, const std::map<std::string, unsigned>& names, std::uint8_t& out) {
  out = 0;
  if (s == "NONE") return true;
  std::stringstream ss(s);
  std::string part;
  while (std::getline(ss, part, '+')) {
    const auto it = names.find(part);
    if (it == names.end()) return false;
    out = static_cast<std::uint8_t>(out | it->second);
  }
  return true;
}

// key value pairs after the first word: "radio range_km 60 latency_ms 8 ..."
bool keyValues(Ctx& c, const std::vector<std::string>& w, std::map<std::string, std::string>& kv) {
  if (w.size() % 2 != 1) return c.fail("'" + w[0] + "' takes key value pairs");
  for (std::size_t i = 1; i + 1 < w.size(); i += 2) kv[w[i]] = w[i + 1];
  return true;
}

bool num(Ctx& c, std::map<std::string, std::string>& kv, const char* key, double& out, double scale = 1.0) {
  const auto it = kv.find(key);
  if (it == kv.end()) return true;
  double v = 0;
  if (!toDouble(it->second, v)) return c.fail(std::string("bad number for ") + key);
  out = v * scale;
  kv.erase(it);
  return true;
}

bool noneLeft(Ctx& c, const std::map<std::string, std::string>& kv) {
  if (kv.empty()) return true;
  return c.fail("unknown key '" + kv.begin()->first + "'");
}

bool meshKey(Ctx& c, mesh::MeshConfig& cfg, const std::string& key, const std::string& val) {
  double v = 0;
  if (!toDouble(val, v)) return c.fail("bad number for mesh " + key);
  const auto u32 = [&](std::uint32_t& f) { f = static_cast<std::uint32_t>(v); };
  if (key == "alert_backup_delay_ms") {
    u32(cfg.alert_backup_delay_ms);
  } else if (key == "heartbeat_period_ms") {
    u32(cfg.heartbeat_period_ms);
  } else if (key == "peer_timeout_ms") {
    u32(cfg.peer_timeout_ms);
  } else if (key == "observation_max_age_ms") {
    u32(cfg.observation_max_age_ms);
  } else if (key == "frame_max_age_ms") {
    u32(cfg.frame_max_age_ms);
  } else if (key == "max_diversion_fraction") {
    cfg.max_diversion_fraction = v;
  } else if (key == "advisory_max_cep_m") {
    cfg.advisory_max_cep_m = v;
  } else if (key == "advisory_min_conf") {
    cfg.advisory_min_conf = v;
  } else if (key == "retask_confirm_ms") {
    u32(cfg.retask_confirm_ms);
  } else if (key == "advisory_bearing_hold_ms") {
    u32(cfg.advisory_bearing_hold_ms);
  } else if (key == "advisory_min_bearings") {
    cfg.advisory_min_bearings = static_cast<unsigned>(v);
  } else if (key == "hysteresis_bonus") {
    cfg.hysteresis_bonus = static_cast<float>(v);
  } else if (key == "secondary_source_novelty") {
    cfg.secondary_source_novelty = v;
  } else {
    return c.fail("unknown mesh key '" + key + "'");
  }
  return true;
}

}  // namespace

bool loadScenario(const std::string& path, Scenario& sc, std::string& error) {
  std::ifstream in(path);
  if (!in) {
    error = path + ": cannot open";
    return false;
  }
  sc = Scenario{};
  sc.path = path;
  Ctx c{path, 0, error};
  const std::map<std::string, unsigned> sensors = {{"RF", 1}, {"EO", 2}, {"IR", 4}, {"AN", 8}};
  const std::map<std::string, unsigned> links = {{"LOS", 1}, {"LEO", 2}, {"GEO", 4}};
  std::set<int> ids;
  std::string raw;
  while (std::getline(in, raw)) {
    ++c.line;
    const std::string text = raw.substr(0, raw.find('#'));
    std::istringstream ls(text);
    std::vector<std::string> w;
    for (std::string t; ls >> t;) w.push_back(t);
    if (w.empty()) continue;
    const std::string& k = w[0];
    double d = 0;
    long n = 0;

    if (k == "name" && w.size() == 2) {
      sc.name = w[1];
    } else if (k == "duration" && w.size() == 2 && toDouble(w[1], d) && d > 0) {
      sc.duration_s = d;
    } else if (k == "seed" && w.size() == 2 && toInt(w[1], n)) {
      sc.seed = static_cast<std::uint32_t>(n);
    } else if (k == "origin" && w.size() == 3) {
      double lat = 0, lon = 0;
      if (!toDouble(w[1], lat) || !toDouble(w[2], lon)) return c.fail("origin <lat> <lon>");
      sc.origin = mesh::GeoPoint{lat, lon, 0.0f};
    } else if (k == "altitude" && w.size() == 2 && toDouble(w[1], d)) {
      sc.altitude_m = d;
    } else if (k == "speeds" && w.size() == 4) {
      if (!toDouble(w[1], sc.route_speed) || !toDouble(w[2], sc.dash_speed) || !toDouble(w[3], sc.rejoin_speed)) {
        return c.fail("speeds <route> <dash> <rejoin> (m/s)");
      }
    } else if (k == "radio") {
      std::map<std::string, std::string> kv;
      if (!keyValues(c, w, kv)) return false;
      auto& r = sc.radio;
      if (const auto it = kv.find("terrain"); it != kv.end()) {
        r.terrain_masking = it->second == "on";
        kv.erase(it);
      }
      if (!num(c, kv, "range_km", r.range_m, 1000) || !num(c, kv, "latency_ms", r.latency_ms) ||
          !num(c, kv, "jitter_ms", r.jitter_ms) || !num(c, kv, "loss_good", r.loss_good) ||
          !num(c, kv, "loss_bad", r.loss_bad) || !num(c, kv, "p_gb", r.p_good_to_bad) ||
          !num(c, kv, "p_bg", r.p_bad_to_good) || !num(c, kv, "skew_ms", r.clock_skew_ms) || !noneLeft(c, kv)) {
        return false;
      }
    } else if (k == "df") {
      std::map<std::string, std::string> kv;
      if (!keyValues(c, w, kv)) return false;
      auto& f = sc.df;
      if (!num(c, kv, "range_km", f.range_m, 1000) || !num(c, kv, "sigma_deg", f.sigma_deg) ||
          !num(c, kv, "period_s", f.period_s) || !num(c, kv, "mp_prob", f.mp_prob) ||
          !num(c, kv, "mp_bias_deg", f.mp_bias_deg) || !num(c, kv, "mp_sigma_deg", f.mp_sigma_deg) ||
          !num(c, kv, "mp_min_s", f.mp_min_s) || !num(c, kv, "mp_max_s", f.mp_max_s) ||
          !num(c, kv, "mp_low_elev_deg", f.mp_low_elev_deg) || !num(c, kv, "mp_low_elev_factor", f.mp_low_elev_factor) ||
          !noneLeft(c, kv)) {
        return false;
      }
    } else if (k == "imaging") {
      std::map<std::string, std::string> kv;
      if (!keyValues(c, w, kv)) return false;
      double cep = sc.img.cep_m;
      if (!num(c, kv, "passive_km", sc.img.passive_m, 1000) || !num(c, kv, "cued_km", sc.img.cued_m, 1000) ||
          !num(c, kv, "sigma_m", sc.img.sigma_m) || !num(c, kv, "cep_m", cep) || !noneLeft(c, kv)) {
        return false;
      }
      sc.img.cep_m = static_cast<float>(cep);
    } else if (k == "clutter") {
      std::map<std::string, std::string> kv;
      if (!keyValues(c, w, kv)) return false;
      if (const auto it = kv.find("class"); it != kv.end()) {
        if (!parseClass(it->second, sc.clutter.cls)) return c.fail("unknown class " + it->second);
        kv.erase(it);
      }
      double conf = sc.clutter.conf;
      if (!num(c, kv, "per_drone_hour", sc.clutter.per_drone_hour) || !num(c, kv, "conf", conf) || !noneLeft(c, kv)) {
        return false;
      }
      sc.clutter.conf = static_cast<float>(conf);
    } else if (k == "c2") {
      std::map<std::string, std::string> kv;
      if (!keyValues(c, w, kv)) return false;
      if (!num(c, kv, "los_ms", sc.c2.latency_ms[0]) || !num(c, kv, "leo_ms", sc.c2.latency_ms[1]) ||
          !num(c, kv, "geo_ms", sc.c2.latency_ms[2]) || !num(c, kv, "jitter_ms", sc.c2.jitter_ms) ||
          !num(c, kv, "ack_s", sc.c2.operator_ack_s) || !noneLeft(c, kv)) {
        return false;
      }
    } else if (k == "hill" && w.size() == 5) {
      Hill h;
      if (!toDouble(w[1], h.e) || !toDouble(w[2], h.n) || !toDouble(w[3], h.height_m) || !toDouble(w[4], h.radius_m)) {
        return c.fail("hill <e_km> <n_km> <height_m> <radius_km>");
      }
      h.e *= 1000;
      h.n *= 1000;
      h.radius_m *= 1000;
      sc.hills.push_back(h);
    } else if (k == "ridge" && w.size() == 7) {
      Ridge r;
      if (!toDouble(w[1], r.e1) || !toDouble(w[2], r.n1) || !toDouble(w[3], r.e2) || !toDouble(w[4], r.n2) ||
          !toDouble(w[5], r.height_m) || !toDouble(w[6], r.width_m)) {
        return c.fail("ridge <e1_km> <n1_km> <e2_km> <n2_km> <height_m> <width_km>");
      }
      r.e1 *= 1000;
      r.n1 *= 1000;
      r.e2 *= 1000;
      r.n2 *= 1000;
      r.width_m *= 1000;
      sc.ridges.push_back(r);
    } else if (k == "drone" && (w.size() == 8 || w.size() == 9)) {
      DroneSpec ds;
      double hours = 0;
      if (!toInt(w[1], n) || n < 1 || n > 0xFFFF) return c.fail("drone id 1..65535");
      ds.id = static_cast<int>(n);
      if (w[2] == "ROUTE") {
        ds.mode = mesh::DroneMode::PACK_ROUTING;
      } else if (w[2] == "ACTIVE") {
        ds.mode = mesh::DroneMode::ACTIVE_MISSION;
      } else {
        return c.fail("drone mode is ROUTE or ACTIVE");
      }
      if (!parseMask(w[3], sensors, ds.sensors)) return c.fail("sensors like RF+EO+IR+AN");
      if (!parseMask(w[4], links, ds.links)) return c.fail("links like LOS+LEO+GEO or NONE");
      if (!toDouble(w[5], hours) || !toDouble(w[6], ds.e) || !toDouble(w[7], ds.n)) {
        return c.fail("drone <id> <mode> <sensors> <links> <endurance_h> <e_km> <n_km> [bingo_s]");
      }
      if (w.size() == 9 && !toDouble(w[8], ds.bingo_s)) return c.fail("bad bingo_s");
      ds.endurance_s = hours * 3600;
      ds.e *= 1000;
      ds.n *= 1000;
      if (!ids.insert(ds.id).second) return c.fail("drone id used twice");
      sc.drones.push_back(ds);
    } else if (k == "emitter" && w.size() >= 7) {
      EmitterSpec e;
      e.label = w[1];
      if (!toInt(w[2], n) || n < 0) return c.fail("bad signature");
      e.sig = static_cast<std::uint32_t>(n);
      if (!toDouble(w[3], e.e) || !toDouble(w[4], e.n) || !toDouble(w[5], e.on_s)) {
        return c.fail("emitter <label> <sig> <e_km> <n_km> <on_s> <class> [thermal] [off=<s>] [mast=<m>]");
      }
      if (!parseClass(w[6], e.cls)) return c.fail("unknown class " + w[6]);
      for (std::size_t i = 7; i < w.size(); ++i) {
        if (w[i] == "thermal") {
          e.thermal = true;
        } else if (w[i].rfind("off=", 0) == 0 && toDouble(w[i].substr(4), d)) {
          e.off_s = d;
        } else if (w[i].rfind("mast=", 0) == 0 && toDouble(w[i].substr(5), d)) {
          e.mast_m = d;
        } else {
          return c.fail("unknown emitter option " + w[i]);
        }
      }
      e.e *= 1000;
      e.n *= 1000;
      for (const auto& o : sc.emitters) {
        if (o.label == e.label) return c.fail("emitter label used twice");
      }
      sc.emitters.push_back(e);
    } else if (k == "benign" && w.size() == 2 && toInt(w[1], n)) {
      sc.benign.push_back(static_cast<std::uint32_t>(n));
    } else if (k == "at" && w.size() >= 3 && toDouble(w[1], d)) {
      ScriptEvent ev;
      ev.t = d;
      const std::string& what = w[2];
      if (what == "lose_verifier" && w.size() == 4) {
        ev.kind = ScriptKind::LOSE_VERIFIER;
        ev.label = w[3];
      } else if (what == "lose_drone" && w.size() == 4 && toInt(w[3], n)) {
        ev.kind = ScriptKind::LOSE_DRONE;
        ev.drone = static_cast<int>(n);
      } else if (what == "dismiss" && w.size() == 4) {
        ev.kind = ScriptKind::DISMISS;
        ev.label = w[3];
      } else if (what == "partition" && w.size() == 4) {
        ev.kind = ScriptKind::PARTITION;
        std::stringstream ss(w[3]);
        for (std::string part; std::getline(ss, part, ',');) {
          if (!toInt(part, n)) return c.fail("partition <id,id,...>");
          ev.group_b.push_back(static_cast<int>(n));
        }
      } else if (what == "heal" && w.size() == 3) {
        ev.kind = ScriptKind::HEAL;
      } else {
        return c.fail("unknown script action '" + what + "'");
      }
      sc.script.push_back(ev);
    } else if (k == "mesh" && w.size() == 3) {
      if (!meshKey(c, sc.cfg, w[1], w[2])) return false;
    } else if (k == "mc") {
      std::map<std::string, std::string> kv;
      if (!keyValues(c, w, kv)) return false;
      double trials = sc.mc_trials;
      if (!num(c, kv, "trials", trials) || !num(c, kv, "jitter_km", sc.mc_jitter_m, 1000) ||
          !num(c, kv, "jitter_on_s", sc.mc_jitter_on_s) || !noneLeft(c, kv)) {
        return false;
      }
      sc.mc_trials = static_cast<int>(trials);
    } else if (k == "expect" && w.size() >= 2) {
      Expect x;
      x.kind = w[1];
      x.text = text.substr(text.find(w[1]));
      while (!x.text.empty() && (x.text.back() == ' ' || x.text.back() == '\t')) x.text.pop_back();
      const auto arg = [&](std::size_t i) { return i < w.size() ? w[i] : std::string(); };
      static const std::set<std::string> labelled = {"advisory_once", "priority_once", "priority_after_advisory",
                                                     "reassigned_after_loss", "quiet", "dismissal_learned",
                                                     "advisory_within", "priority_within"};
      if (labelled.count(x.kind) != 0) {
        x.label = arg(2);
        if (x.label.empty()) return c.fail("expect " + x.kind + " needs an emitter label");
        if ((x.kind == "advisory_within" || x.kind == "priority_within") && !toDouble(arg(3), x.value)) {
          return c.fail("expect " + x.kind + " <label> <seconds>");
        }
        if (x.kind == "priority_once" && w.size() == 4 && !toDouble(arg(3), x.value)) {
          return c.fail("expect priority_once <label> [max_err_m]");
        }
      } else if (x.kind == "never_tasked") {
        if (!toInt(arg(2), n)) return c.fail("expect never_tasked <drone>");
        x.drone = static_cast<int>(n);
      } else if (x.kind == "max_fcs_requests" || x.kind == "max_false_alerts") {
        if (!toDouble(arg(2), x.value)) return c.fail("expect " + x.kind + " <n>");
      } else if (x.kind != "all_released" && x.kind != "converged") {
        return c.fail("unknown expectation '" + x.kind + "'");
      }
      sc.expects.push_back(x);
    } else {
      return c.fail("cannot read '" + text + "'");
    }
  }

  c.line = 0;
  if (sc.name.empty()) return c.fail("no name");
  if (sc.drones.size() < 2 || sc.drones.size() > mesh::kMaxPackSize) {
    return c.fail("a pack has 2 to " + std::to_string(mesh::kMaxPackSize) + " drones");
  }
  if (!mesh::isValid(sc.cfg)) return c.fail("mesh settings fail isValid()");
  for (auto& e : sc.emitters) {
    bool benign = false;
    for (const auto s : sc.benign) benign = benign || (s == e.sig && s != 0);
    e.alertable = !benign && mesh::eventSeverity(e.cls) >= sc.cfg.advisory_min_severity;
  }
  const auto known = [&](const std::string& label) {
    for (const auto& e : sc.emitters) {
      if (e.label == label) return true;
    }
    return false;
  };
  for (const auto& x : sc.expects) {
    if (!x.label.empty() && !known(x.label)) return c.fail("expect names unknown emitter " + x.label);
    if (x.kind == "never_tasked" && ids.count(x.drone) == 0) return c.fail("expect names unknown drone");
  }
  for (const auto& s : sc.script) {
    if (!s.label.empty() && !known(s.label)) return c.fail("script names unknown emitter " + s.label);
  }
  return true;
}

}  // namespace sim
