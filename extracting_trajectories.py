import os
import sys
import tempfile
import subprocess
from pathlib import Path
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import comtypes.client

# --- DOCX to PDF ---
def docx_to_pdf(docx_path):
    word = comtypes.client.CreateObject('Word.Application')
    word.Visible = False
    try:
        doc = word.Documents.Open(docx_path)
        pdf_path = os.path.splitext(docx_path)[0] + '.pdf'
        doc.SaveAs(pdf_path, FileFormat=17)  # 17 = wdFormatPDF
        doc.Close()
        return pdf_path
    except Exception as e:
        print("DOCX to PDF error:", e)
        return None
    finally:
        word.Quit()

# --- PDF to SVG ---
def pdf_to_svg(pdf_path):
    INKSCAPE_PATH = r"C:\Program Files\Inkscape\bin\inkscape.exe"
    svg_path = tempfile.NamedTemporaryFile(delete=False, suffix=".svg").name
    cmd = [
        INKSCAPE_PATH,
        pdf_path,
        "--export-type=svg",
        f"--export-filename={svg_path}"
    ]
    try:
        subprocess.run(cmd, check=True, capture_output=True)
        return svg_path
    except subprocess.CalledProcessError as e:
        print("Inkscape PDF to SVG error:", e.stderr.decode())
        return None


# --- Write simple SVG from TXT ---
def write_svg_from_txt(lines, svg_path, font="Arial", size_pt=12, scale=1.0):
    size_pt_scaled = size_pt * scale
    width_mm = 210
    height_mm = 297
    margin_mm = 10
    pt_to_mm = 0.3528
    line_spacing_mm = size_pt_scaled * pt_to_mm * 1.2

    svg_lines = []
    start_y = margin_mm + 15
    for i, line in enumerate(lines):
        y_pos = start_y + i * line_spacing_mm
        if y_pos > (height_mm - margin_mm):
            break
        safe_text = line.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        svg_line = f'<text x="{margin_mm}" y="{y_pos}" font-family="{font}" font-size="{size_pt_scaled}pt">{safe_text}</text>'
        svg_lines.append(svg_line)

    svg_content = f'''<?xml version="1.0" standalone="no"?>
<svg width="{width_mm}mm" height="{height_mm}mm" viewBox="0 0 {width_mm} {height_mm}" xmlns="http://www.w3.org/2000/svg">
  {'\n  '.join(svg_lines)}
</svg>
'''
    with open(svg_path, "w", encoding="utf-8") as f:
        f.write(svg_content)


# --- TXT to SVG ---
def txt_to_svg(txt_path, font="Arial", size_pt=12):
    with open(txt_path, "r", encoding="utf-8") as f:
        lines = [line.strip() for line in f if line.strip()]
    svg_path = tempfile.NamedTemporaryFile(delete=False, suffix=".svg").name
    write_svg_from_txt(lines, svg_path, font, size_pt)
    return svg_path

def preview_svg(svg_path, root):
    # Set A4 at 96 DPI
    a4_width_px = 794  # 210 mm
    a4_height_px = 1123  # 297 mm

    png_path = tempfile.NamedTemporaryFile(delete=False, suffix=".png").name
    INKSCAPE_PATH = r"C:\Program Files\Inkscape\bin\inkscape.exe"

    cmd = [
        INKSCAPE_PATH,
        svg_path,
        "--export-type=png",
        f"--export-filename={png_path}",
        f"--export-width={a4_width_px}",
        f"--export-height={a4_height_px}"
    ]

    try:
        subprocess.run(cmd, check=True, capture_output=True)
    except subprocess.CalledProcessError as e:
        print("Inkscape preview error:", e.stderr.decode())
        messagebox.showerror("Inkscape Error", "Failed to generate preview. Check Inkscape installation.")
        return None

    img = Image.open(png_path)
    photo = ImageTk.PhotoImage(img)

    canvas = tk.Canvas(root, width=a4_width_px, height=a4_height_px)
    canvas.pack(padx=10, pady=10)

    canvas.create_image(0, 0, anchor="nw", image=photo)
    canvas.image = photo

    return canvas


def svg_to_dxf(svg_path, dxf_path):
    # Use Inkscape CLI to export DXF
    INKSCAPE_PATH = r"C:\Program Files\Inkscape\bin\inkscape.exe"  # full exe path

    cmd = [
        INKSCAPE_PATH,
        svg_path,
        "--export-type=dxf",
        f"--export-filename={dxf_path}",
        "--export-text-to-path"

    ]

    try:
        subprocess.run(cmd, check=True, capture_output=True)
        print(f"DXF saved to: {dxf_path}")
        return dxf_path
    except subprocess.CalledProcessError as e:
        print("Inkscape SVG to DXF error:", e.stderr.decode())
        return None


def main(file_path):
    if not file_path.exists():
        print(f"File '{file_path}' does not exist.")
        return

    ext = file_path.suffix.lower()

    if ext == ".docx":
        pdf_path = docx_to_pdf(str(file_path))
        if not pdf_path:
            print("Failed DOCX->PDF.")
            return
        svg_path = pdf_to_svg(pdf_path)
        if not svg_path:
            print("Failed PDF->SVG.")
            return
    elif ext == ".txt":
        lines = []
        with open(file_path, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f if line.strip()]
        svg_path = tempfile.NamedTemporaryFile(delete=False, suffix=".svg").name
        write_svg_from_txt(lines, svg_path)
    else:
        print("Unsupported file type.")
        return

    # Export to DXF
    dxf_path = file_path.with_suffix(".dxf")
    result_dxf = svg_to_dxf(svg_path, str(dxf_path))
    if not result_dxf:
        print("Failed to create DXF file.")
    else:
        print(f"DXF file created at: {result_dxf}")

    import tkinter as tk
    root = tk.Tk()
    root.title("SVG Preview")
    preview_svg(svg_path, root)
    root.mainloop()


if __name__ == "__main__":
    if sys.platform != "win32":
        print("This script runs only on Windows.")
        sys.exit(1)

    try:
        import comtypes
    except ImportError:
        print("Please install 'comtypes' with: pip install comtypes")
        sys.exit(1)

    script_dir = Path(__file__).parent.resolve()
    filename = input("Enter the DOCX or TXT file name (with extension): ").strip()
    file_path = (script_dir / filename).resolve()

    main(file_path)

