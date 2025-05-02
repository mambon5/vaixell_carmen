# Substituir tots els caràcters Unicode no compatibles (ex: cometes tipogràfiques) per equivalents ASCII

import unicodedata

def clean_text(text):
    return unicodedata.normalize('NFKD', text).encode('ascii', 'ignore').decode('ascii')

# Crear nou PDF netejat
pdf = FPDF()
pdf.add_page()
pdf.set_auto_page_break(auto=True, margin=15)
pdf.set_font("Arial", size=12)

# Títol principal net
pdf.set_font("Arial", 'B', 14)
pdf.multi_cell(0, 10, clean_text("Tutorial basic d'Excel per a principiants: Analisi de dades"), align='C')
pdf.ln(5)

# Afegir les seccions amb text netejat
for title, content in sections:
    pdf.set_font("Arial", 'B', 12)
    pdf.multi_cell(0, 10, clean_text(title))
    pdf.set_font("Arial", size=12)
    for line in content.split('\n'):
        pdf.multi_cell(0, 8, clean_text(line))
    pdf.ln(3)

# Guardar el PDF
pdf_output_path = "/mnt/data/Tutorial_Basic_Excel.pdf"
pdf.output(pdf_output_path)

pdf_output_path
