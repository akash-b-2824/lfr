from machine import Pin,PWM
from time import sleep
import network
import socket
import heapq

SSID = "A"
PASSWORD = "77777777"

station = network.WLAN(network.STA_IF)
station.active(True)
station.connect(SSID, PASSWORD)

while not station.isconnected():
    pass

print("Connected, IP:", station.ifconfig()[0])


html = """
<!DOCTYPE html>
<html>
<body>
    <h2>Enter Locations</h2>
    <form action="/" method="GET">
        call Location: <input type="text" name="final"><br>
        <input type="submit" value="Submit">
    </form>
</body>
</html>
"""



#line follower meow meow

    










# Setup Web Server
def web_page():
    return html

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', 80))
s.listen(5)
def con():
    while True:
        conn, addr = s.accept()
        request = conn.recv(1024).decode()
        
        if "GET /?final=" in request:
            parts = request.split(" ")[1] 
            params = parts.split("?")[1] 
            params_dict = {k: v for k, v in (x.split("=") for x in params.split("&"))}
            final_loc = params_dict.get("final", "Unknown")
            print(f"Final Location: {final_loc}")
            return final_loc
        
        response = web_page()
        conn.send("HTTP/1.1 200 OK\nContent-Type: text/html\n\n" + response)
        conn.close()
start_node='81'
initial_direction='s'
end_node=""
def mainn():       
    f=con()
    global initial_direction,start_node,end_node
    def navigate_robot(start, end, initial_direction, graph):
        def dijkstra(graph, start, end):
            queue = [(0, start, [])]
            visited = set()
            while queue:
                cost, node, path = heapq.heappop(queue)
                if node not in visited:
                    visited.add(node)
                    path = path + [node]
                    if node == end:
                        return path
                    for direction, neighbor in graph[node].items():
                        if neighbor not in visited:
                            heapq.heappush(queue, (cost + 1, neighbor, path))
            return None

        def get_command(current_node, next_node, current_direction):
            for direction, node in graph[current_node].items():
                if node == next_node:
                    target_direction = direction
                    break
            else:
                return None

            if current_direction == target_direction:
                return 'S'
            elif (current_direction == 'n' and target_direction == 'e') or \
                 (current_direction == 'e' and target_direction == 's') or \
                 (current_direction == 's' and target_direction == 'w') or \
                 (current_direction == 'w' and target_direction == 'n'):
                return 'R'
            elif (current_direction == 'n' and target_direction == 'w') or \
                 (current_direction == 'w' and target_direction == 's') or \
                 (current_direction == 's' and target_direction == 'e') or \
                 (current_direction == 'e' and target_direction == 'n'):
                return 'L'
            else:
                return 'U'

        path = dijkstra(graph, start, end)
        if not path:
            return "No path found!"

        commands = []
        current_direction = initial_direction
        for i in range(len(path) - 1):
            current_node = path[i]
            next_node = path[i + 1]
            command = get_command(current_node, next_node, current_direction)
            commands.append(command)
            if command == 'R':
                if current_direction == 'n':
                    current_direction = 'e'
                elif current_direction == 'e':
                    current_direction = 's'
                elif current_direction == 's':
                    current_direction = 'w'
                elif current_direction == 'w':
                    current_direction = 'n'
            elif command == 'L':
                if current_direction == 'n':
                    current_direction = 'w'
                elif current_direction == 'w':
                    current_direction = 's'
                elif current_direction == 's':
                    current_direction = 'e'
                elif current_direction == 'e':
                    current_direction = 'n'
            elif command == 'U':
                if current_direction == 'n':
                    current_direction = 's'
                elif current_direction == 's':
                    current_direction = 'n'
                elif current_direction == 'e':
                    current_direction = 'w'
                elif current_direction == 'w':
                    current_direction = 'e'
        commands.append("S")
        return commands,current_direction


    graph = {
        '11': {'s': '21'},
        '12': {'s': '22'},
        '13': {'s': '23'},
        '14': {'s': '25'},
        '21': {'n': '11', 'e': '22', 's': '31'},
        '22': {'n': '12', 's': '32', 'w': '21','e':'23'},
        '23': {'n': '13', 's': '33', 'w': '22'},
        '24': {'e': '25','s':'34'},
        '25': {'n': '14','s': '35','e': '26','w': '24'},
        '26': {'w': '25'},
        '31': {'n': '21', 'e': '32'},
        '32': {'n': '22','e':'33','w': '31'},
        '33': {'n': '23','s': '42','e': '34','w': '32'},
        '34': {'n': '24','s': '43','e': '35','w': '33'},
        '35': {'w':'34','n':'25','e': '36','s': '44'},
        '36': {'w':'31','s':'45'},
        '41': {'s': '52'},
        '42': {'n': '33','s':'53','e': '43'},
        '43': {'w': '42','n': '34','e': '44'},
        '44': {'n': '35','s': '64','e': '45','w': '43'},
        '45': {'w':'44','n':'36'},
        '51': {'e': '52'},
        '52': {'s': '61','e':'53','n': '41','w': '51'},
        '53': {'w': '52','n': '42','s': '62'},
        '54': {'s': '65'},
        '61': {'n': '52'},
        '62': {'s': '73','e':'63','n': '53'},
        '63': {'w': '62','e': '64','s': '84'},
        '64': {'n': '44','s': '74','e': '65','w': '63'},
        '65': {'w':'64','n':'54','s': '75'},
        '71': {'s': '81', 'e': '72'},
        '72': {'s': '82','e':'73','w': '71'},
        '73': {'w': '72','s': '83','n': '62'},
        '74': {'n': '64','s': '85','e': '75'},
        '75': {'w':'74','n':'65','s': '86'},
        '81': {'n': '71'},
        '82': {'n': '72'},
        '83': {'n': '73'},
        '84': {'n': '63'},
        '85': {'n':'74'},
        '86': {'n':'75'},

    }
    end_node =f.split("%2C")[0].lower()
    commands,initial_direction = navigate_robot(start_node, end_node, initial_direction, graph)
    start_node=end_node
    return commands

#lfr functions and initialization
lmf=Pin(4,Pin.OUT)
lmb=Pin(2,Pin.OUT)
lmp=PWM(Pin(15))

# #right motor connect 
rmf=Pin(22,Pin.OUT)
rmb=Pin(21,Pin.OUT)
rmp=PWM(Pin(23))




#recurance prevention

#pwm frequency
lmp.freq(1500)
rmp.freq(1500)

#ir connect
BL=Pin(34,Pin.IN,Pin.PULL_DOWN)
FL=Pin(35,Pin.IN,Pin.PULL_DOWN)
FS=Pin(32,Pin.IN,Pin.PULL_DOWN)
FR=Pin(33,Pin.IN,Pin.PULL_DOWN)
BR=Pin(25,Pin.IN,Pin.PULL_DOWN)

#motor forward
def forward():
    lmp.duty_u16(65000)
    rmp.duty_u16(65000)
    lmf.value(1)
    rmf.value(1)
    lmb.value(0)
    rmb.value(0)

#motor turn left
def turn_right():
    lmp.duty_u16(000)
    rmp.duty_u16(65000)
    lmf.value(0)
    rmf.value(1)
    lmb.value(1)
    rmb.value(0)

#motor turn right
def turn_left():
    lmp.duty_u16(65000)
    rmp.duty_u16(000)
    lmf.value(1)
    rmf.value(0)
    lmb.value(0)
    rmb.value(1)

#motor tilt left
def tilt_right():
    lmp.duty_u16(45000)
    rmp.duty_u16(65000)
    lmf.value(1)
    rmf.value(1)
    lmb.value(0)
    rmb.value(0)

#motor tilt right
def tilt_left():
    lmp.duty_u16(65000)
    rmp.duty_u16(45000)
    lmf.value(1)
    rmf.value(1)
    lmb.value(0)
    rmb.value(0)
    
#motor uturn left
def uturn_right():
    lmp.duty_u16(52000)
    rmp.duty_u16(52000)
    lmf.value(0)
    rmf.value(1)
    lmb.value(1)
    rmb.value(0)
#motor uturn right
def uturn_left():
    lmp.duty_u16(65000)
    rmp.duty_u16(65000)
    lmf.value(1)
    rmf.value(0)
    lmb.value(0)
    rmb.value(1)

#motor stop
def stop():
    lmf.value(0)
    rmf.value(0)
    lmb.value(0)
    rmb.value(0)
def reverse():
    lmp.duty_u16(65000)
    rmp.duty_u16(65000)
    lmf.value(0)
    rmf.value(0)
    lmb.value(1)
    rmb.value(1)
    
def readsensor():
    global fl,fs,fr,br,bl
    fl = FL.value()   
    fs = FS.value()
    fr = FR.value()
    br = BR.value()
    bl = BL.value()
def st():
    global path,rec,lef,reg,k
    path="".join(mainn())
    path=path.lower()
    print(path)
    path=path.replace('l','z')
    path=path.replace("r","l")
    path=path.replace('z','r')
    rec=True
    lef=False
    reg=False
    if not (path[0]=='u' or path[0]=='s'):
        reverse()
        sleep(.05)
        stop()
        sleep(.1)
        reverse()
        sleep(1)
        stop()
        sleep(.5)
    if path[0]=='s':
        path=path[1:]
    path+='j'
    k=-len(path)
while True:
    st()

    def preo(g):
        global rig
        global lef,k
        if path[g]=='r':
            lef=True
            rig=False
        elif path[g]=='j':
            stop()
            sleep(1)
            st()
        elif path[g]=='u':
            stop()
            sleep(.4)
            forward()
            sleep(.3)
            uturn_left()
            sleep(1.3)
            stop()
            sleep(.3)
            k+=1
        elif path[g]=='s':
            lef=False
            rig=False
        else:
            lef=False
            rig=True
    #main loop

    #main loop
    while True:
        preo(k)
        #initialize short cuts
        readsensor()

        #condition strict left
        if (bl==1 or br==1)and(rec and (not rig)and(not lef)):
            rec=False
            k+=1
            forward()
            sleep(.3)
        elif bl==1 and rec and lef:
            forward()
            sleep(.4)
            uturn_left()
            k+=1
            rec=False
            sleep(.5)
        elif br==1 and rec and rig:
            forward()
            sleep(.4)
            uturn_right()
            k+=1
            rec=False
            sleep(.5)
        elif fl==0 and fs==1 and fr==0:
            forward()
            rec=True
        elif fl==1 and fs==1 and fr==0:
            tilt_left()
            rec=True
        elif fl==1 and fs==0 and fr==0:turn_left()
        elif fl==0 and fs==1 and fr==1:
            tilt_right()
            rec=True
        elif fl==0 and fs==0 and fr==1:turn_right()
        elif fl==1 and fs==1 and fr==1:forward()
        sleep(0.001)     






