import gzip
import shutil
from pathlib import Path

# Ejecuta este script desde la RAÍZ del repo: .../simple_video_server
base = Path("frontend/dist")
out  = Path("frontend/gzipped")

mapping = {
    "index.html": "index.html.gz",
    "loading.jpg": "loading.jpg.gz",
    "favicon.ico": "favicon.ico.gz",
    "assets/index.js": "assets/index.js.gz",
    "assets/index.css": "assets/index.css.gz",
}

# Asegura carpetas destino
(out / "assets").mkdir(parents=True, exist_ok=True)

for src_rel, dst_rel in mapping.items():
    src = base / src_rel
    dst = out / dst_rel

    if not src.exists():
        raise FileNotFoundError(f"NO existe: {src}")

    dst.parent.mkdir(parents=True, exist_ok=True)

    with open(src, "rb") as f_in, gzip.open(dst, "wb", compresslevel=9) as f_out:
        shutil.copyfileobj(f_in, f_out)

    print(f"OK  {src_rel}  ->  {dst_rel}  ({dst.stat().st_size} bytes)")

print("\nListo. Ya tienes los .gz en frontend/gzipped/ para que CMake los embeba.")
