import os
import re
import markdown
import subprocess

base_dir = "/Users/ritwik/htdocs/robotic-hand-cad"
md_path = os.path.join(base_dir, "ENGINEERING_GUIDE.md")
html_path = os.path.join(base_dir, "ENGINEERING_GUIDE.html")
pdf_path = os.path.join(base_dir, "ENGINEERING_GUIDE.pdf")

with open(md_path, "r", encoding="utf-8") as f:
    text = f.read()

# Replace raw LaTeX math formulas with clean typography
text = text.replace(r"$\alpha_x = +0.85\text{ rad } (+48.7^\circ)$", "<b>α<sub>x</sub> = +0.85 rad (+48.7°)</b>")
text = text.replace(r"$\beta_y = -0.35\text{ rad } (-20.1^\circ)$", "<b>β<sub>y</sub> = -0.35 rad (-20.1°)</b>")
text = text.replace(r"$\gamma_z = +0.50\text{ rad } (+28.6^\circ)$", "<b>γ<sub>z</sub> = +0.50 rad (+28.6°)</b>")
text = text.replace(r"$\varnothing 2.0\text{ mm}$", "Ø2.0 mm")
text = text.replace(r"$\varnothing 1.5 - 2.0\text{ mm}$", "Ø1.5 – 2.0 mm")
text = text.replace(r"$\varnothing 10–14\text{ mm}$", "Ø10 – 14 mm")
text = text.replace(r"($1500\,\mu\text{s}$)", "(1500 µs)")
text = text.replace(r"$90^\circ - 130^\circ$", "90° – 130°")

# Convert markdown to html with tables extension
html_body = markdown.markdown(text, extensions=['tables', 'fenced_code'])

styled_html = f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Biomechanical Anthropomorphic Robotic Hand - Engineering Guide</title>
<style>
    @page {{
        size: A4;
        margin: 18mm 16mm 18mm 16mm;
        @bottom-right {{
            content: "Page " counter(page);
            font-size: 9pt;
            color: #64748b;
        }}
    }}
    body {{
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        color: #1e293b;
        line-height: 1.55;
        font-size: 10.5pt;
        background: #fff;
    }}
    .header-banner {{
        border-bottom: 3px solid #0284c7;
        padding-bottom: 12px;
        margin-bottom: 20px;
    }}
    h1 {{
        color: #0f172a;
        font-size: 20pt;
        margin: 0 0 6px 0;
        font-weight: 800;
        letter-spacing: -0.5px;
    }}
    .subtitle {{
        font-size: 11pt;
        color: #0284c7;
        font-weight: 600;
        margin: 0;
    }}
    h2 {{
        color: #0369a1;
        font-size: 14pt;
        border-bottom: 1px solid #e2e8f0;
        padding-bottom: 4px;
        margin-top: 22px;
        margin-bottom: 10px;
        page-break-after: avoid;
    }}
    h3 {{
        color: #0f172a;
        font-size: 11.5pt;
        margin-top: 14px;
        margin-bottom: 6px;
        page-break-after: avoid;
    }}
    p, li {{
        font-size: 10pt;
        color: #334155;
    }}
    table {{
        width: 100%;
        border-collapse: collapse;
        margin: 12px 0 16px 0;
        font-size: 9.5pt;
        page-break-inside: avoid;
    }}
    th, td {{
        border: 1px solid #cbd5e1;
        padding: 6px 9px;
        text-align: left;
    }}
    th {{
        background-color: #f1f5f9;
        color: #0f172a;
        font-weight: 600;
    }}
    tr:nth-child(even) {{
        background-color: #f8fafc;
    }}
    code {{
        background: #f1f5f9;
        color: #0369a1;
        padding: 1px 4px;
        border-radius: 4px;
        font-family: Menlo, Monaco, Consolas, monospace;
        font-size: 9pt;
    }}
    pre {{
        background: #0f172a;
        color: #f8fafc;
        padding: 10px 14px;
        border-radius: 6px;
        font-size: 8.5pt;
        overflow-x: auto;
        page-break-inside: avoid;
    }}
    ul, ol {{
        padding-left: 20px;
        margin-top: 4px;
        margin-bottom: 10px;
    }}
    li {{
        margin-bottom: 3px;
    }}
    hr {{
        border: none;
        border-top: 1px solid #e2e8f0;
        margin: 18px 0;
    }}
</style>
</head>
<body>
<div class="header-banner">
    <h1>Biomechanical Anthropomorphic Robotic Hand</h1>
    <p class="subtitle">Complete Engineering, Actuation & Fabrication Guide (27 DoF Servo Architecture)</p>
</div>
{html_body}
</body>
</html>
"""

# Remove the redundant h1 markdown if present
styled_html = styled_html.replace("<h1>Biomechanical Anthropomorphic Robotic Hand: Engineering &amp; Fabrication Guide</h1>", "")

with open(html_path, "w", encoding="utf-8") as f:
    f.write(styled_html)

chrome_path = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
cmd = [
    chrome_path,
    "--headless=new",
    "--disable-gpu",
    "--no-pdf-header-footer",
    f"--print-to-pdf={pdf_path}",
    html_path
]

res = subprocess.run(cmd, capture_output=True, text=True)
if res.returncode == 0:
    print(f"Regenerated PDF: {pdf_path} ({os.path.getsize(pdf_path)} bytes)")
else:
    print("Error:", res.stderr)
