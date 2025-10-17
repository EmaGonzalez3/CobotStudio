import pandas as pd
import numpy as np
import os
import re

def string_to_array(s):
    if isinstance(s, str):
        return np.fromstring(s, sep=' ')
    return s

def procesar_y_guardar_html(df, titulo_tabla, ruta_salida_html):
    df_procesado = df.copy()
    columns_to_convert = ['q_esp', 'q_alc', 'err_q', 'pose_esp', 'pose_alc', 'err_pose', 'tb_check']
    for col in columns_to_convert:
        if col in df_procesado.columns:
            df_procesado[col] = df_procesado[col].apply(string_to_array)
    html_tabla = df_procesado.to_html(index=False, justify='center', border=1)
    html_completo = f"""
    <html>
    <head>
    <style>
      body {{ font-family: sans-serif; }}
      table {{ border-collapse: collapse; margin: 25px 0; font-size: 0.9em; }}
      th, td {{ padding: 12px 15px; border: 1px solid #dddddd; text-align: center; vertical-align: middle; }}
      thead tr {{ background-color: #009879; color: #ffffff; text-align: center; }}
      tbody tr {{ border-bottom: 1px solid #dddddd; }}
      tbody tr:nth-of-type(even) {{ background-color: #f3f3f3; }}
    </style>
    </head>
    <body>
    <h2>{titulo_tabla}</h2>
    {html_tabla}
    </body>
    </html>
    """
    with open(ruta_salida_html, 'w', encoding='utf-8') as f:
        f.write(html_completo)
    print(f"-> Tabla HTML consolidada guardada en: {ruta_salida_html}")

# 1. Definir rutas base
directorio_script = os.path.dirname(os.path.abspath(__file__))
ruta_resultados = os.path.join(directorio_script, 'Resultados')
subcarpetas_entrada = ['ModelTestPreCal', 'ModelTestPostCal']

print("--- Iniciando recolección de datos para consolidación ---")

# Listas para guardar los DataFrames de cada tipo
lista_dfs_pre = []
lista_dfs_post = []

# 2. Bucle para leer y recolectar todos los datos
for nombre_subcarpeta in subcarpetas_entrada:
    ruta_subcarpeta_actual = os.path.join(ruta_resultados, nombre_subcarpeta)
    print(f"Analizando carpeta: {nombre_subcarpeta}")
    if not os.path.isdir(ruta_subcarpeta_actual): continue
    for nombre_archivo in os.listdir(ruta_subcarpeta_actual):
        if nombre_archivo.startswith('test_coords_log_v') and nombre_archivo.endswith('.csv'):
            tipo_resultado = "Pre" if "Pre" in nombre_subcarpeta else "Post"
            match = re.search(r'_v(\d+)\.csv', nombre_archivo)
            if not match: continue
            numero_version = match.group(1)
            print(f"  Recolectando: {nombre_archivo}")
            try:
                ruta_completa_entrada = os.path.join(ruta_subcarpeta_actual, nombre_archivo)
                df = pd.read_csv(ruta_completa_entrada, sep=';')
                df['version'] = int(numero_version)
                if tipo_resultado == "Pre":
                    lista_dfs_pre.append(df)
                else:
                    lista_dfs_post.append(df)
            except Exception as e:
                print(f"ERROR al procesar el archivo {nombre_archivo}: {e}")

# Procesar y generar la tabla para todos los resultados PRE-calibración
if lista_dfs_pre:
    df_pre_completo = pd.concat(lista_dfs_pre, ignore_index=True)
    
    df_pre_completo = df_pre_completo.sort_values(by=['version', 'punto_n']).reset_index(drop=True)
    
    ruta_html_pre = os.path.join(ruta_resultados, 'Resultados_Consolidado_Pre.html')
    titulo_pre = "Resultados Consolidados Pre-Calibración"
    procesar_y_guardar_html(df_pre_completo, titulo_pre, ruta_html_pre)
else:
    print("No se encontraron datos PRE para generar la tabla.")

# Procesar y generar la tabla para todos los resultados POST-calibración
if lista_dfs_post:
    df_post_completo = pd.concat(lista_dfs_post, ignore_index=True)

    df_post_completo = df_post_completo.sort_values(by=['version', 'punto_n']).reset_index(drop=True)

    ruta_html_post = os.path.join(ruta_resultados, 'Resultados_Consolidado_Post.html')
    titulo_post = "Resultados Consolidados Post-Calibración"
    procesar_y_guardar_html(df_post_completo, titulo_post, ruta_html_post)
else:
    print("No se encontraron datos POST para generar la tabla.")

print("\n--- Proceso completado. ---")