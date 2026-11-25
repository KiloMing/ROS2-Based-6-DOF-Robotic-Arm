#!/usr/bin/env python3
"""Import the exact V2 preview archive without overwriting the existing model."""
import argparse
import hashlib
from pathlib import Path, PurePosixPath
import shutil
import tempfile
import zipfile

SHA256 = "0806d49c67a1879e11bf5905f77d1527e082f54b6e22953e151499231d78fc3f"

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    package = repo / "arm_ws/src/arm_description_cad"
    docs = repo / "docs/cad_v2"
    if package.exists() or docs.exists():
        parser.error("Destination exists; back it up before importing.")
    with args.archive.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    if digest != SHA256:
        parser.error("Archive SHA-256 does not match the original V2 ZIP.")
    with tempfile.TemporaryDirectory() as temp:
        temp = Path(temp)
        with zipfile.ZipFile(args.archive) as archive:
            for entry in archive.infolist():
                path = PurePosixPath(entry.filename)
                if path.is_absolute() or ".." in path.parts:
                    parser.error("Unsafe archive path.")
            archive.extractall(temp)
        source = temp / "arm_cad_dxf_v2"
        model = source / "arm_description_cad"
        if not (model / "urdf/arm_cad.urdf").is_file():
            parser.error("Model missing from archive.")
        for kind in ("visual", "collision"):
            if len(list((model / "meshes" / kind).glob("*.stl"))) != 10:
                parser.error("Incomplete STL set.")
        shutil.copytree(model, package)
        try:
            shutil.copytree(source / "analysis", docs)
            shutil.copy2(source / "README.md", docs / "README.md")
        except Exception:
            shutil.rmtree(package)
            if docs.exists():
                shutil.rmtree(docs)
            raise
    print("Imported V2 preview package and analysis. No hardware settings changed.")
    print("Review the imported files before git add, commit and push.")

if __name__ == "__main__":
    main()
