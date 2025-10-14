import pandas as pd
import numpy as np
import io
import webbrowser # Librería para abrir el navegador
import os

# El nombre de tu archivo real
filename = 'test_coords_log_v3.csv' 

df = pd.read_csv(filename, sep=';')

# Lista de columnas que contienen nuestros vectores de texto
columns_to_convert = ['q_esp', 'q_alc', 'err_q', 'pose_esp', 'pose_alc', 'err_pose', 'tb_check']

# Función auxiliar que toma un string "1.0 2.0 3.0" y devuelve un array de numpy [1., 2., 3.]
def string_to_array(s):
    if isinstance(s, str):
        return np.fromstring(s, sep=' ')
    return s # Devuelve el valor original si no es un string (ej. NaN)

# Aplicamos la función a cada columna que lo necesita
for col in columns_to_convert:
    df[col] = df[col].apply(string_to_array)

html_tabla = df.to_html(index=False, justify='center', border=1)

# Le damos algo de estilo a la tabla
html_completo = f"""
<html>
<head>
<style>
  body {{ font-family: sans-serif; }}
  table {{ border-collapse: collapse; margin: 25px 0; font-size: 0.9em; }}
  th, td {{ padding: 12px 15px;
    border: 1px solid #dddddd;
    text-align: center;
    vertical-align: middle;
  }}
  thead tr {{ background-color: #009879; color: #ffffff; text-align: center; }}
  tbody tr {{ border-bottom: 1px solid #dddddd; }}
  tbody tr:nth-of-type(even) {{ background-color: #f3f3f3; }}
</style>
</head>
<body>
<h2>Resultados del Ensayo del Modelo</h2>
{html_tabla}
</body>
</html>
"""

# Guardar el HTML en un archivo
output_html_file = 'tabla_resultados.html'
with open(output_html_file, 'w') as f:
    f.write(html_completo)

print(f"La tabla se ha guardado como un archivo HTML en: {output_html_file}")

# webbrowser.open('file://' + os.path.realpath(output_html_file))