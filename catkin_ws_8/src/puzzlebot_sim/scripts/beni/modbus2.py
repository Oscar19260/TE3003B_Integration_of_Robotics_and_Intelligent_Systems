import socket
import binascii
import PySimpleGUI as sg

# Crear la interfaz grafica de usuario con PySimpleGUI
sg.theme('DefaultNoMoreNagging')
layout = [
    [sg.Text('Direccion IP del dispositivo:'), sg.InputText(key='ip')],
    [sg.Text('Puerto de acceso:'), sg.InputText(key='port')],
    [sg.Text('Cadena hexadecimal a enviar:'), sg.InputText(key='data')],
    [sg.Button('Enviar datos'), sg.Button('Salir')]
]
window = sg.Window('Envio de datos Modbus TCP/IP', layout)

# Funcion para enviar datos a traves de Modbus TCP/IP
def send_modbus_tcp(ip, port, data):
    try:
        # Crear un socket TCP/IP
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Conectar el socket al puerto donde el dispositivo esta escuchando
        server_address = (ip, int(port))
        sock.connect(server_address)
        
        # Convertir la cadena hexadecimal a bytes
        hex_data = binascii.unhexlify(data)
        
        # Enviar los datos al dispositivo
        sock.sendall(hex_data)
        
        # Recibir la respuesta del dispositivo
        response = sock.recv(1024)
        
        # Imprimir la respuesta recibida
        print('Respuesta recibida:', binascii.hexlify(response))
        
    finally:
        # Cerrar el socket
        sock.close()

# Bucle principal de la interfaz grafica de usuario
while True:
    event, values = window.read()
    
    if event == sg.WINDOW_CLOSED or event == 'Salir':
        break
        
    if event == 'Enviar datos':
        #ip = values['ip']
        #port = values['port']
        #data = values['data']
        
        # Enviar los datos a traves de Modbus TCP/IP
        #send_modbus_tcp(ip, port, data)
        send_modbus_tcp("192.168.1.174", 502, '00010002000D19DB0F4940F366DF4000000000')
        send_modbus_tcp("192.168.1.173", 502, '00010002000D19DB0F4940F366DF4000000000')
        send_modbus_tcp("192.168.1.153", 502, '00010002000D19DB0F4940F366DF4000000000')
        send_modbus_tcp("192.168.1.182", 502, '00010002000D19DB0F4940F366DF4000000000')
        send_modbus_tcp("192.168.1.175", 502, '00010002000D19DB0F4940F366DF4000000000')
        send_modbus_tcp("192.168.1.165", 502, '00010002000D19DB0F4940F366DF4000000000')
# Cerrar la ventana de la interfaz grafica de usuario
window.close()
