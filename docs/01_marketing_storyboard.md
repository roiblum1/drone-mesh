# Deliverable 1 — Marketing storyboard and image prompts

**Message in one line:** *Six aircraft, one operator — and now all six are working.*

The storyboard has six panels. They work as a single hero image, as a six-slide build, or as a row of thumbnails on one slide. Numbers in the callouts come from the repo's 6-drone simulation (`sim/pack_sim.cpp`), so they are consistent with the code. Label them "simulated scenario" on any slide.

All aircraft are shown **unarmed**: a sensor payload under the fuselage, nothing on the wings. That matches the system, which detects, localises, verifies and alerts.

## Look of the aircraft (use in every image)

A generic 150 kg-class fixed-wing UAV, not any real product:

- Pusher propeller driven by a small combustion engine, with a visible exhaust stub.
- Twin tail booms joined by a horizontal stabiliser.
- Long, slender, high-aspect-ratio wing, about 6 m span, in matte light grey.
- Small EO/IR turret under the nose, a low blade antenna on the spine for the line-of-sight link, and a small dome behind the wing root for the satellite link.
- No national markings, no weapons, no logos. A thin teal stripe on the boom is the only brand colour.

## Colour language (keeps all six panels reading as one system)

| Meaning | Colour | Drawn as |
|---|---|---|
| Mesh link, drone to drone | Teal / cyan | Thin glowing arcs, small travelling "packet" dots |
| Detection bearing (RF) | Amber | Hairlines from a drone to the ground, converging |
| Position uncertainty (CEP) | Amber, translucent | Ellipse on the ground that shrinks panel by panel |
| Verified alert | White | Thin line climbing to the satellites |
| Operator-controlled drone | Warm white ring | One ring on one aircraft |

---

## The six panels

### Panel 1 — "One operator. One working sensor."

- **Visual:** Six UAVs in line abreast over open terrain at altitude, about ten kilometres apart (shown tight for the shot). One aircraft has a bright warm-white ring: the operator-controlled drone. The other five are slightly dimmed, with a faint dotted route line ahead of them. Quiet, wide, empty sky.
- **Headline:** *Today, only one aircraft is on the job.*
- **Caption:** The operator can actively control one drone. The rest fly the route with their sensors and processors idle.
- **Callout chip:** `1 of 6 sensor suites in use`

### Panel 2 — "The mesh wakes the pack"

- **Visual:** The same formation. Every aircraft now has a soft teal halo. Thin teal arcs connect neighbours, with small packet dots travelling along them. Each drone shows a faint conical sensor sweep towards the ground. The operator-controlled drone keeps its warm-white ring; it shares its sensor readings but is never re-tasked by the mesh.
- **Headline:** *Every aircraft listens. Every aircraft shares.*
- **Caption:** An airborne peer-to-peer mesh, run by the Model 8270 on each drone, turns passive route-flyers into a distributed sensor.
- **Callout chip:** `6 of 6 sensor suites contributing`

### Panel 3 — "First contact"

- **Visual:** Low on the ground, a small amber glow: a distress beacon with a thermal signature at the edge of a forest. One drone's sweep clips it and an amber bearing line draws from that drone to the ground. A teal arc flares from that drone to its neighbours.
- **Headline:** *One hears it. All of them know.*
- **Caption:** The first bearing is shared across the mesh in the next mesh cycle. No operator in the loop yet.
- **Callout chip:** `Shared within one 0.1 s mesh cycle (simulated)`

### Panel 4 — "Bearings cross"

- **Visual:** Four more amber lines join, all converging on the glow. On the ground a large translucent ellipse tightens around it. A small label near the ellipse reads `CEP 708 m` (rendered in the slide, not by the image model).
- **Headline:** *Five drones triangulate what one could only hear.*
- **Caption:** Bearing-only readings from several vantage points fuse into a position, with no drone leaving formation.
- **Callout chip:** `5 observers · CEP ≈ 0.7 km · early-warning sent`

### Panel 5 — "The best-placed two peel off"

- **Visual:** Two aircraft bank away from the formation on different headings, one from the east and one from the north, so the sensor cones hit the source from two angles. The rest of the pack keeps flying line abreast. The operator-controlled aircraft is unaffected. One aircraft in the line carries a small amber fuel icon: it keeps scanning but is not diverted.
- **Headline:** *The pack decides who goes. The rest keep watching.*
- **Caption:** Drones are chosen by time-to-target, available sensor type, fuel margin and viewing angle. At most two fifths of the pack ever leave the route.
- **Callout chip:** `2 verifiers · different angles · 3 keep scanning`

### Panel 6 — "A verified alert, not a video feed"

- **Visual:** The ellipse has collapsed to a bright dot. A single white packet climbs to a LEO satellite, then to a higher GEO satellite, then down to a ground control centre. Inside, the operator's wall display shows a map pin and a short alert card. The mesh arcs in the sky have dimmed to low glow.
- **Headline:** *The operator gets an answer, not a stream.*
- **Caption:** Two sensor types confirm the position. One compact alert goes to the operator over the best available link.
- **Callout chip:** `PRIORITY alert · CEP ≈ 70 m · ~1.7 kbit/s average mesh load per drone`

---

## Master text-to-image prompt (hero image, panels 3–6 in one frame)

> **Prompt**
>
> Cinematic wide aerial scene, late-afternoon golden light with a thin high haze. Six identical unarmed fixed-wing UAVs, each about 150 kg: slender high-aspect-ratio matte-grey wings, twin tail booms, a rear-mounted pusher propeller with a small combustion engine, a small EO/IR turret under the nose, a low blade antenna on the spine. They fly in a loose line-abreast formation over forested hills at the edge of a dry plain. Between the aircraft, thin glowing teal holographic mesh links arc from drone to drone, with tiny travelling data packets, clean and minimal like a technical diagram laid over a photograph. On the ground, a small amber glow (a distress beacon with a thermal signature). Five thin amber bearing lines run from five of the drones down to that glow and cross there, with a translucent amber ellipse on the ground tightening around it. Two of the drones have banked away from the formation on different headings, their sensor cones touching the glow from two angles; the other four continue straight. From the lead area of the formation a single thin white line climbs to two small low-earth-orbit satellites and then to a larger geostationary satellite high in the upper corner, curving back down toward a distant ground control centre with softly lit windows at the horizon. Photorealistic aircraft detail, subtle volumetric light, shallow atmospheric depth, restrained palette of grey, teal, amber and white, uncluttered composition with empty space in the upper left for a slide title, 16:9, ultra-detailed, sharp focus.

> **Negative prompt**
>
> weapons, missiles, bombs, pylons, gun pods, camouflage, military insignia, flags, roundels, jet engines, extra propellers, extra wings, cartoon, anime, sci-fi spaceships, neon cyberpunk city, readable text, logos, watermark, people, crowded composition, lens flare overload

**Tips**

1. Generate the picture with no text. Add the labels in PowerPoint or Figma. Image models garble small lettering.
2. If the mesh links come out too heavy, add "thin hairline links, 1 px, low opacity". If the drones come out as quadcopters, repeat "fixed-wing, slender long wing".
3. For a six-slide build, generate panel 1 first, then reuse its seed or image reference and add one idea per panel.

## Midjourney variant

```
cinematic wide aerial, golden hour, six identical unarmed 150 kg fixed-wing UAVs with long slender grey wings, twin tail booms, rear pusher propeller, small EO/IR turret, flying line abreast over forested hills, thin glowing teal holographic mesh links arcing between the drones with tiny data packets, five thin amber bearing lines converging on a small amber glow on the ground with a tightening translucent ellipse, two drones banking away on different headings, a single thin white line climbing to two low-earth-orbit satellites and a geostationary satellite, distant ground control centre at the horizon, restrained grey teal amber white palette, empty space upper left, photorealistic, volumetric haze --ar 16:9 --style raw --stylize 250 --no weapons missiles insignia flags text logo cartoon
```

(Adjust the version flag to the Midjourney version you use.)

## Short variant (DALL·E / Imagen-style, under 900 characters)

> A wide, photorealistic aerial view at golden hour of six identical unarmed fixed-wing UAVs, about 150 kg each, with long slender grey wings, twin tail booms and a rear propeller, flying in a loose line over forested hills. Thin glowing teal lines link the aircraft to each other like a clean technical overlay. Five thin amber lines run from the aircraft down to a small amber glow on the ground and cross on it, with a translucent amber ellipse around the glow. Two of the aircraft have banked away on different headings, aimed at the glow from two angles. A single thin white line rises from the formation to two small satellites and a larger satellite high above, curving back to a distant ground control centre on the horizon. Restrained palette of grey, teal, amber and white. No text, no weapons, no insignia. 16:9.

## Alternate shots

**A. Hero, low angle ("the constellation")**

> Low-angle view from the ground looking up at six unarmed fixed-wing UAVs crossing a clear late-afternoon sky in a loose line, each with long slender grey wings, twin tail booms and a rear propeller. Thin teal holographic arcs connect them like a constellation. Two high satellites glow faintly in the upper corner, joined to the formation by one thin white line. Photorealistic, restrained palette, calm and confident, large negative space for a headline, 16:9. No weapons, no insignia, no text.

**B. Ground control centre ("the operator gets an answer")**

> Interior of a modern, softly lit ground control centre at dusk. One operator seated at a console, back to the camera, facing a large wall display that shows a clean map with a single glowing pin, a small ellipse around it and a short alert card with no readable text. To the side, a smaller screen shows a simple diagram of six aircraft linked by thin teal lines, one of them highlighted with a warm-white ring. Calm, uncluttered, cinematic, shallow depth of field, grey, teal, amber and white palette, 16:9. No logos, no flags, no readable text.

## Slide copy (ready to paste)

| Slide | Title | One line |
|---|---|---|
| 1 | One operator. One working sensor. | Today, only the drone the operator controls contributes to the mission. |
| 2 | The mesh wakes the pack | Peer-to-peer links turn every route-flyer into a distributed sensor. |
| 3 | One hears it. All of them know. | A first bearing reaches every aircraft in the next mesh cycle. |
| 4 | Five drones triangulate what one could only hear | Cooperative direction-finding locates a source without anyone leaving formation. |
| 5 | The pack decides who goes | Best-placed drones verify from different angles; the rest keep scanning. |
| 6 | The operator gets an answer, not a stream | One verified alert, over the best available link. |

*Footnote for every slide using the callouts:* "Figures from a 6-drone software simulation of the mesh logic; not flight-test results."
