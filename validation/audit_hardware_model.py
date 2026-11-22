"""Check saved hardware wiring and source preservation without MATLAB or USB.

Run from any folder: python3 /absolute/path/to/validation/audit_hardware_model.py
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def props(element, tag="P"):
    return {p.get("Name"): p.text for p in element.findall(tag)}


def system(archive, name="root"):
    return ET.fromstring(archive.read(f"simulink/systems/system_{name}.xml"))


def blocks(tree):
    return {b.get("Name"): b for b in tree.findall("Block")}


def child(archive, block):
    return ET.fromstring(archive.read("simulink/systems/" + block.find("System").get("Ref") + ".xml"))


def wiring(tree):
    names = {b.get("SID"): b.get("Name") for b in tree.findall("Block")}
    result = {}
    for line in tree.findall("Line"):
        source = props(line).get("Src")
        destinations = [p.text for p in line.iter("P") if p.get("Name") == "Dst"]
        if destinations:
            assert source, "Dangling line source"
            sid, port = source.split("#out:")
            source = (names[sid], int(port))
            for destination in destinations:
                sid, port = destination.split("#in:")
                endpoint = (names[sid], int(port))
                assert endpoint not in result, "Multiply driven input"
                result[endpoint] = source
    return result


def references(archive):
    for name in archive.namelist():
        if name.startswith("simulink/systems/") and name.endswith(".xml"):
            for block in ET.fromstring(archive.read(name)).findall("Block"):
                if "quarc_library" in props(block).get("SourceBlock", ""):
                    yield props(block)["SourceBlock"]


def audit():
    supplied = ROOT / "real_hardware"
    checksums = json.loads((supplied / "source_checksums.json").read_text())
    for name, expected in checksums.items():
        assert hashlib.sha256((supplied / name).read_bytes()).hexdigest() == expected, name
    with zipfile.ZipFile(supplied / "lin_feedback_EMPTY.slx") as original:
        original_blocks = blocks(system(original))
        original_io = blocks(child(original, original_blocks["System"]))
        for suffix in ("", "_R2021a"):
            with zipfile.ZipFile(ROOT / f"simulink/aero2_hardware_cascade{suffix}.slx") as adapted:
                tree = system(adapted)
                top = blocks(tree)
                io = blocks(child(adapted, top["AERO 2 hardware"]))
                assert len(list(references(adapted))) == 4
                # All HIL settings, channel conversions and rate estimator settings
                # remain those supplied by the professor, including shutdown zeros.
                assert props(top["HIL Initialize"].find("InstanceData")) == props(
                    original_blocks["HIL Initialize1"].find("InstanceData"))
                for name, block in original_io.items():
                    if block.get("BlockType") == "Outport":
                        continue  # Two misleading speed output labels were corrected.
                    assert block.get("BlockType") == io[name].get("BlockType"), name
                    for key in ("Gain", "Indices", "SourceBlock"):
                        assert props(block).get(key) == props(io[name]).get(key), (name, key)
                    if block.find("InstanceData") is not None:
                        assert props(block.find("InstanceData")) == props(io[name].find("InstanceData")), name
                wires = wiring(tree)
                expected = {
                    ("AERO 2 hardware", 1): ("Motor voltages", 1),
                    ("AERO 2 hardware", 2): ("Motor voltages", 2),
                    ("Measured angles", 1): ("AERO 2 hardware", 1),
                    ("Measured angles", 2): ("AERO 2 hardware", 4),
                    ("Measured rates", 1): ("AERO 2 hardware", 3),
                    ("Measured rates", 2): ("AERO 2 hardware", 5),
                    ("Main tachometer filter", 1): ("AERO 2 hardware", 7),
                    ("Tail tachometer filter", 1): ("AERO 2 hardware", 8),
                }
                for destination, source in expected.items():
                    assert wires[destination] == source, destination
                config = props(ET.fromstring(adapted.read("simulink/configSet0.xml")), ".//P")
                assert config["FixedStep"] == "0.002"
                assert config["SystemTargetFile"] == "quarc_win64.tlc"
            with zipfile.ZipFile(ROOT / f"simulink/experiments/aero2_hardware_cascade_offline{suffix}.slx") as offline:
                assert not list(references(offline)), "Hardware block in offline surrogate"
                wires = wiring(system(offline))
                assert wires[("Measured angles", 1)] == ("Offline AERO 2 surrogate", 1)
                assert wires[("Measured angles", 2)] == ("Offline AERO 2 surrogate", 4)
    print("PASS: four supplied files unchanged; HIL settings/channel mapping preserved;")
    print("hardware and R2021a export wired; both offline models contain zero QUARC references.")


if __name__ == "__main__":
    audit()
