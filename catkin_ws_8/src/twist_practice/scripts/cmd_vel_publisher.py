#!/usr/bin/env python
                        # Primer linea de codigo (anterior) --> Indica al OS el interprete de Python a utilizar
import rospy                        

from geometry_msgs.msg import Twist # Importamos el tipo de dato Twist, o mensajes de tipo geometry_msgs/Twist (vel. lineal y angular)
                                    # Del PAQUETE geometry_msgs, en la CARPETA msg, importamos el TIPO DE DATO, MENSAJE o ESTRUCTURA Twist

if __name__ =="__main__":

    rospy.init_node('cmd_vel_publisher', anonymous=True)    # (1st arg, 2nd arg) = (nombre del nodo, usar modo anonimo para evitar nodos con el mismo nombre)
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # (1st arg, 2nd arg, 3rd arg) = (nombre del topico meta, tipo de mensaje, size del buffer interno del topico)
                                                            # queue_size = Numero de mensajes que queremos que sea capaz de almacenar el topico antes de comenzar a perder info
    rate = rospy.Rate(10)       # 10hz  # Frecuencia a la que se publica
    
    f_msg = Twist()             # Instanciamos un objeto de clase Twist
    
    while not rospy.is_shutdown():
    
        f_msg.angular.z = 0.5   # Counterclockwise --> Corresponde a velocidad angular Z positiva, hacia la izquierda
                                # Simplemente, le asignamos al atributo de clase un valor coherente para la Wz
                                
        pub.publish(f_msg)      # Publicamos el mensaje generado
        
        rate.sleep()


