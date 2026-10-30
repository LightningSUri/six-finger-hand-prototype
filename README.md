# Six-Finger Hand Prototype

An experimental tendon-driven hand concept exploring five parallel fingers and a sixth, angled thumb. This repository contains a parameterized CadQuery generator, individual STL parts, STEP geometry, and assembly documentation.

**Current stage: V3 CAD bench prototype.** The design has not been physically printed or tested. It is not a fitted or validated wearable prosthesis.

![Six-finger hand CAD assembly](exports/reference/ASSEMBLY_PREVIEW.png)

[View the lightweight 3D assembly](exports/reference/GITHUB_PREVIEW_DO_NOT_PRINT.stl) · [Download the STEP assembly](exports/reference/FULL_ASSEMBLY.step) · [Read the assembly guide](docs/ASSEMBLY_GUIDE.md)

## Design question

Could an additional digit provide useful grasp configurations for everyday tasks? This project explores that question through CAD iteration, tendon routing, and modular finger mechanisms. Any improvement over a conventional five-digit arrangement remains a hypothesis to be tested; no comparative grasp-performance results are claimed.

## Model features

- **Six distinct digits:** five parallel front fingers and one side thumb oriented at 45° to the main finger direction.
- **Three components per digit:** fixed base, proximal link, and distal link, with two modeled pin joints per digit.
- **19 structural parts:** one palm chassis and 18 digit components.
- **Contoured palm:** domed upper surface, tapered wrist end, and hollow underside.
- **Tendon routing:** six nominal 2.2 mm guide paths with open cord spans across moving joints and removable interfaces.
- **Return anchors:** dorsal slots for elastic return loops.
- **Adjustable CAD parameters:** dimensions, layout, pin clearance, and clevis side clearance are defined in the Python source.

The script generates the model from primitives without requiring an input CAD file. The supplied STEP assembly is positioned for viewing; it does not contain native FreeCAD motion constraints or sketch history.

## Start here

### View the hand

Open [the lightweight assembly STL](exports/reference/GITHUB_PREVIEW_DO_NOT_PRINT.stl) in GitHub's 3D viewer. If the viewer does not load, download the file and open it locally.

For CAD inspection, download the repository using **Code → Download ZIP**, extract it, and open `exports/reference/FULL_ASSEMBLY.step` in FreeCAD. The STL preview is a lower-resolution viewing asset; it is not a print file.

### Inspect individual parts

The [print_parts folder](exports/print_parts/) contains one STL per structural component. The [step folder](exports/step/) contains the corresponding CAD solids. Read the [assembly guide](docs/ASSEMBLY_GUIDE.md) before preparing a test print.

Print only individual component STLs. Neither the full assembly nor the posed-digit reference files are print-in-place models. Hardware, cords, elastic, slicer settings, and printer-specific G-code are not supplied.

### Run the generator

Use a Python environment compatible with CadQuery 2.7. The existing workflow used Python 3.12; Python 3.11/3.12 are the intended starting versions.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python six_finger_hand.py --out generated_exports
```

The new output folder keeps the supplied reference exports intact. The script runs in ordinary Python, not in FreeCAD's macro console. CAD operations can take several minutes or longer depending on the machine.

Optional, potentially slow collision checks:

```bash
python six_finger_hand.py --out generated_exports --check-motion
```

The lightweight GitHub preview is a separate publication asset; the main script generates the full-resolution reference assembly and individual parts.

## Key fit parameters

| Parameter | Current nominal CAD value |
| --- | ---: |
| Hinge pin diameter | 3.0 mm |
| Radial clearance around pin | 0.3 mm |
| Resulting hinge-hole diameter | 3.6 mm |
| Clevis side clearance | 0.3 mm per side |
| Center lug width | 8.0 mm |
| Fork opening | 8.6 mm |
| Tendon guide diameter | 2.2 mm |

A 0.3 mm **radial** gap around a 3.0 mm pin requires a 3.6 mm hole. These values are initial design allowances, not guaranteed printed fits. Test a hinge pair in the intended material and orientation before a complete build.

## Verification and limitations

| Check or milestone | Status |
| --- | --- |
| Six digits / 19 structural parts present | Confirmed in supplied exports |
| Individual printable CAD parts | Reported valid single solids |
| Printable STL edge incidence | Passed at 0.0001 mm vertex-welding precision |
| Final full-assembly collision checks | Not completed; motion clearance remains unverified |
| Final standalone script regenerated end-to-end | Not completed |
| Physical printing, threading, fit, load, or endurance tests | Not performed |
| Wearable fitting or clinical evaluation | Not performed |

Final exports reused a previously generated palm solid and applied the final conduit-tip trimming operation. The supplied source contains that operation and has no dependency on the cached solid. See [dimensions and check status](exports/dimensions_and_checks.json) and [mesh checks](exports/mesh_checks.json) for the recorded results.

The design does not include physical travel stops, a wearable socket, motors, electronics, or a control system. Thumb opposition, full motion range, tendon friction, retention, and useful grasp performance require further development and testing. Single-bolt digit bases are not positively keyed against rotation.

## Next development steps

- [ ] Complete a fresh end-to-end generation and reproducibility check.
- [ ] Complete assembly collision and tendon-path checks over the intended motion range.
- [ ] Print and evaluate one joint and one complete finger.
- [ ] Test channel threading, cord abrasion, and elastic return.
- [ ] Add joint stops and improve base retention.
- [ ] Assemble and test the complete bench mechanism.
- [ ] Compare selected tasks with the sixth digit enabled and parked.
- [ ] Record measured results and revise the design accordingly.

## Development and acknowledgments

This is a student-led assistive-device design exploration. The repository distinguishes modeled features from experimentally demonstrated function.

## Feedback

Suggestions about mechanism geometry, tendon routing, FDM preparation, and test methods are welcome through GitHub Issues. Include the component filename, a screenshot or sketch, and a proposed change where possible.

## Licensing

No license has been selected for this publication package. Public visibility alone does not make the repository open source. An explicit license can be added after deciding how the code, CAD files, and documentation may be reused.
