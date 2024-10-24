import psycopg2

# Conectarse a la base de datos PostgreSQL
conn = psycopg2.connect(
    dbname="mqttDB",
    user="caca",
    password="123",
    host="mqtt",  # Cambia a la dirección de tu servidor PostgreSQL si es necesario
    port="5432"  # Puerto por defecto de PostgreSQL
)

# Crear un cursor para ejecutar consultas SQL
cursor = conn.cursor()

# Ejemplo: Consultar datos de una tabla
cursor.execute("SELECT * FROM nombre_de_tabla;")
rows = cursor.fetchall()
for row in rows:
    print(row)

# Ejemplo: Insertar datos en una tabla
cursor.execute("INSERT INTO nombre_de_tabla (columna1, columna2) VALUES (%s, %s);", (valor1, valor2))
conn.commit()

# Cerrar el cursor y la conexión
cursor.close()
conn.close()
