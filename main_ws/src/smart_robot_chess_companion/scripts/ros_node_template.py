#! /usr/bin/env python
# |_ la prima linea di commento indica che questo file è uno script python
# per lanciare il nodo: rosrun smart_robot_chess_companion ros_node_template.py

# N.B. template per l'implementazione di un nodo ros che agisce sia come publisher che come subscriber, rispetto a topic diversi;
#      si suppone che i dati scambiati mediante topic siano sotto forma di dizionari python 

from smart_robot_chess_companion import utils
import rospy
from std_msgs.msg import String

# funzione per la gestione della ricezione di un nuovo messaggio ros su un determinato topic, 
# automaticamente invocata dal subscriber
def callback_function(message, *args):
    print('==START SUBSCRIBER OUTPUT==')
    # estrazione di un dizionario codificato in stringa JSON dal messaggio ROS
    message_data_dict = utils.dict_from_string_ros_message(message)

    # args è una tupla con un unico elemento dato dalla lista callback_args (vedi sotto)
    args = args[0]
    arg1 = args[0]
    arg2 = args[1]
    print(f'arg1={arg1}, arg2={arg2}')
    
    # a questo punto si possono estrarre i dati da message_data_dict e processarli usando gli eventuali argomenti in args
    # ...

    # ogni messaggio ros ritornato dalla funzione utils.dict_to_string_ros_message(dictionary) include un timestamp
    # che può essere usato come id del messaggio se necessario
    timestamp = message_data_dict['timestamp']
    print(f'Extracted dictionary: {message_data_dict}')
    print('==END SUBSCRIBER OUTPUT==\n')
    
def main_function():

    # inizializzazione nodo ros con nome example_ros_node - anonymous=True aggiunge un id al nome che lo rende univoco
    rospy.init_node('example_ros_node', anonymous=True)
    
    # frequenza di esecuzione del loop while not rospy.is_shutdown() in Herz (vedi sotto)
    rate = rospy.Rate(0.5) # 0.5 => 1 loop ogni 2 secondi
    
    # publisher che scrive messaggi di tipo std_msgs/String nel topic con nome example_topic
    publisher = rospy.Publisher('example_topic', String, queue_size=10)

    # subscriber che legge messaggi di tipo std_msgs/String dal topic con nome example_topic
    # n.b. alla ricezione di un nuovo messaggio, il subscriber chiama la funzione callback_function in un nuovo thread,
    #      passando il messaggio - argomento message di callback_function - e gli altri argomenti della funzione, definiti come
    #      una lista callback_function_args
    # p.s. non è richiesto passare ulteriori argomenti se la funzione di callback non richiede altro che il messaggio ros
    arg1 = 'a1'
    arg2 = 'a2'
    callback_function_args = [arg1, arg2]
    subscriber = rospy.Subscriber(
        'example_topic', 
        String, 
        callback_function, 
        callback_args=callback_function_args
    )
    
    while not rospy.is_shutdown():
        # esempio di pubblicazione di un nuovo messaggio da parte del publisher
        dict = { 'key1': 'value1', 'key2': 'value2', 'key3': 'value3' }
        message = utils.dict_to_string_ros_message(dict)
        publisher.publish(message)

        print('==START PUBLISHER OUTPUT==')
        print(f'Published message: {message}')
        print('==END PUBLISHER OUTPUT==\n')

        # garantisce l'esecuzione del loop con la frequenza specificata in rate
        rate.sleep() 
            
if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass