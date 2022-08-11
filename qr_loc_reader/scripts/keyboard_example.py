from pynput import keyboard
import yaml
qr_dict = {'a':'A'}

def on_press(key):
    print('{0} pressed'.format(key))
    if key == keyboard.KeyCode(char='y'):
        with open('qr_tf.yaml', 'w') as outfile:
            yaml.dump(qr_dict, outfile, default_flow_style=False)
        print("qr_tf coordinates yaml saved! :)")