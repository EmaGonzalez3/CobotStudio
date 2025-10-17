import pandas as pd
import numpy as np
import webbrowser
import os
import re

# --- Funciones de Preparación y Exportación (sin cambios) ---

def preparar_df_completo(df):
    """Procesa un DataFrame completo, convirtiendo las columnas de string a array."""
    df_procesado = df.copy()
    columns_to_convert = ['q_esp', 'q_alc', 'err_q', 'pose_esp', 'pose_alc', 'err_pose', 'tb_check']
    for col in columns_to_convert:
        if col in df_procesado.columns:
            df_procesado[col] = df_procesado[col].apply(lambda s: np.fromstring(s.strip('[]'), sep=' ') if isinstance(s, str) else s)
    return df_procesado

def preparar_df_reporte_errores(df_completo):
    """Toma un DataFrame consolidado y devuelve un DataFrame para el reporte de errores."""
    columnas_reporte = ['punto_n', 'version', 'err_q', 'err_pose', 'tb_check', 'error_rot_fro']
    df_reporte = df_completo[columnas_reporte].copy()
    df_reporte['err_pose'] = df_reporte['err_pose'].apply(lambda s: np.fromstring(s.strip('[]'), sep=' ') if isinstance(s, str) else s)
    
    def calcular_err_tras(pose_array):
        if isinstance(pose_array, np.ndarray) and len(pose_array) >= 3:
            return np.linalg.norm(pose_array[:3])
        return np.nan
        
    df_reporte['err_tras'] = df_reporte['err_pose'].apply(calcular_err_tras)
    cols = ['punto_n', 'version', 'err_q', 'err_pose', 'err_tras', 'tb_check', 'error_rot_fro']
    return df_reporte[cols]

def guardar_df_como_html(df, titulo_tabla, ruta_salida_html):
    """Toma CUALQUIER DataFrame y lo guarda en un archivo HTML."""
    print(f"Exportando a HTML: {titulo_tabla}...")
    # ... (código de la función sin cambios)
    html_tabla = df.to_html(index=False, justify='center', border=1)
    html_completo = f"""
    <html><head><style>
      body {{ font-family: sans-serif; }} table {{ border-collapse: collapse; margin: 25px 0; font-size: 0.9em; }}
      th, td {{ padding: 12px 15px; border: 1px solid #dddddd; text-align: center; vertical-align: middle; }}
      thead tr {{ background-color: #009879; color: #ffffff; text-align: center; }}
      tbody tr {{ border-bottom: 1px solid #dddddd; }} tbody tr:nth-of-type(even) {{ background-color: #f3f3f3; }}
    </style></head><body>
    <h2>{titulo_tabla}</h2>
    {html_tabla}
    </body></html>
    """
    with open(ruta_salida_html, 'w', encoding='utf-8') as f:
        f.write(html_completo)
    print(f"-> Guardado en: {ruta_salida_html}")

# --- Funciones que envuelven las acciones para el menú ---
def accion_exportar_completas(df_pre, df_post, ruta_resultados):
    print("\n--- Exportando Tablas Completas ---")
    df_html_pre = preparar_df_completo(df_pre)
    guardar_df_como_html(df_html_pre, "Resultados Consolidados Pre-Calibración", os.path.join(ruta_resultados, 'Resultados_Consolidado_Pre.html'))
    df_html_post = preparar_df_completo(df_post)
    guardar_df_como_html(df_html_post, "Resultados Consolidados Post-Calibración", os.path.join(ruta_resultados, 'Resultados_Consolidado_Post.html'))

def accion_exportar_reportes(df_pre, df_post, ruta_resultados):
    print("\n--- Exportando Reportes de Error ---")
    guardar_df_como_html(df_pre, "Reporte de Errores Pre-Calibración", os.path.join(ruta_resultados, 'Reporte_Errores_Pre.html'))
    guardar_df_como_html(df_post, "Reporte de Errores Post-Calibración", os.path.join(ruta_resultados, 'Reporte_Errores_Post.html'))

def accion_analisis_consola(df_pre, df_post):
    print("\n--- Realizando Análisis de Error de Traslación ---")
    promedio_err_tras_pre = df_pre['err_tras'].dropna().mean()
    promedio_err_tras_post = df_post['err_tras'].dropna().mean()
    mejora = promedio_err_tras_pre - promedio_err_tras_post
    mejora_porcentual = (mejora / promedio_err_tras_pre) * 100 if promedio_err_tras_pre != 0 else float('inf')
    
    print(f"Promedio de error de traslación (Pre-Calibración):  {promedio_err_tras_pre:.4f}")
    print(f"Promedio de error de traslación (Post-Calibración): {promedio_err_tras_post:.4f}")
    print(f"\nMejora absoluta: {mejora:.4f}")
    print(f"Mejora porcentual: {mejora_porcentual:.2f}%")

# ==============================================================================
# --- Script Principal ---
# ==============================================================================

# 1. Recolección y consolidación de datos (se ejecuta siempre una vez)
directorio_script = os.path.dirname(os.path.abspath(__file__))
ruta_resultados = os.path.join(directorio_script, 'Resultados')
subcarpetas_entrada = ['ModelTestPreCal', 'ModelTestPostCal']
lista_dfs_pre, lista_dfs_post = [], []

print("--- Iniciando recolección y consolidación de datos... ---")
# ... (El bucle de recolección es idéntico) ...
for nombre_subcarpeta in subcarpetas_entrada:
    ruta_subcarpeta_actual = os.path.join(ruta_resultados, nombre_subcarpeta)
    if not os.path.isdir(ruta_subcarpeta_actual): continue
    for nombre_archivo in os.listdir(ruta_subcarpeta_actual):
        if nombre_archivo.startswith('test_coords_log_v') and nombre_archivo.endswith('.csv'):
            tipo_resultado = "Pre" if "Pre" in nombre_subcarpeta else "Post"
            match = re.search(r'_v(\d+)\.csv', nombre_archivo)
            if not match: continue
            try:
                df = pd.read_csv(os.path.join(ruta_subcarpeta_actual, nombre_archivo), sep=';')
                df['version'] = int(match.group(1))
                (lista_dfs_pre if tipo_resultado == "Pre" else lista_dfs_post).append(df)
            except Exception as e:
                print(f"ERROR al recolectar {nombre_archivo}: {e}")

if not lista_dfs_pre or not lista_dfs_post:
    raise SystemExit("No se encontraron suficientes datos para continuar. Proceso abortado.")

df_pre_completo = pd.concat(lista_dfs_pre, ignore_index=True).sort_values(by=['version', 'punto_n'])
df_post_completo = pd.concat(lista_dfs_post, ignore_index=True).sort_values(by=['version', 'punto_n'])

print("--- Datos listos para el análisis. ---")

# 2. Preparar los DataFrames finales para los reportes
df_reporte_pre = preparar_df_reporte_errores(df_pre_completo)
df_reporte_post = preparar_df_reporte_errores(df_post_completo)

# ==============================================================================
# --- Bucle de Menú Interactivo ---
# ==============================================================================
while True:
    print("\n╔═════════════════════════════════════════════════╗")
    print("║                   MENÚ DE ACCIONES                ║")
    print("╠═════════════════════════════════════════════════╣")
    print("║ 1. Exportar Tablas Completas (HTML)               ║")
    print("║ 2. Exportar Reportes de Error (HTML)              ║")
    print("║ 3. Realizar Análisis de Error en Consola          ║")
    print("║ 4. Ejecutar TODAS las acciones anteriores         ║")
    print("║                                                 ║")
    print("║ 0. Salir                                          ║")
    print("╚═════════════════════════════════════════════════╝")
    
    opcion = input("Seleccione una opción: ")

    if opcion == '1':
        accion_exportar_completas(df_pre_completo, df_post_completo, ruta_resultados)
    elif opcion == '2':
        accion_exportar_reportes(df_reporte_pre, df_reporte_post, ruta_resultados)
    elif opcion == '3':
        accion_analisis_consola(df_reporte_pre, df_reporte_post)
    elif opcion == '4':
        accion_exportar_completas(df_pre_completo, df_post_completo, ruta_resultados)
        accion_exportar_reportes(df_reporte_pre, df_reporte_post, ruta_resultados)
        accion_analisis_consola(df_reporte_pre, df_reporte_post)
    elif opcion == '0':
        print("Saliendo del programa.")
        break
    else:
        print("\n¡Opción no válida! Por favor, seleccione un número del menú.")

print("\n--- Proceso finalizado. ---")