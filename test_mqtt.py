# test_mqtt.py
import paho.mqtt.client as mqtt, ssl

def on_connect(client, userdata, flags, rc, properties=None):
    print(f"Connected rc={rc}")
    client.subscribe("#")

def on_message(client, userdata, msg):
    print(f"MSG: {msg.topic} → {msg.payload.decode()}")

def on_disconnect(client, userdata, flags, rc, properties=None):
    print(f"Disconnected rc={rc}")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, transport="websockets")
client.username_pw_set("Botler", "Botler")
client.tls_set_context(ssl.create_default_context())
client.on_connect    = on_connect
client.on_message    = on_message
client.on_disconnect = on_disconnect
client.connect("c22d3035.ala.us-east-1.emqxsl.com", 8084, keepalive=60)
client.loop_forever()