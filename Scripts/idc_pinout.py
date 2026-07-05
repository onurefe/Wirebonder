# These are the noted values from my numbering system.
left_panel = {
'search_plus': [3, 12], 
'search_minus': [3, 13], 
'loop_plus': [2, 12], 
'loop_minus': [2, 13], 
'tail_plus': [1, 11], 
'tail_minus': [1, 13], 
'step_plus': [1, 10], 
'step_minus': [1, 12], 
'up': [2, 8], 
'left': [3, 10], 
'down': [2, 11], 
'right': [1, 9], 
'minus': [2, 10], 
'plus': [2, 9], 
'manual': [3, 11], 
'enter': [3, 8], 
'esc': [3, 9], 
'add': [3, 7], 
'save': [2, 7], 
'load': [1, 8], 
'led_anode': [5], 
'led_cathode': [14]
}

right_panel = {
"test_led": [1, 5],
"high_reset_led": [1, 10],
"setup_led": [1, 6],
"short_led": [1, 8],
"open_led": [1, 9],
"clamp_open_led": [1, 11],
"high_reset": [2, 7],
"light": [3, 7],
"clamp_open": [15, 7],
"setup": [14, 7],
"reset": [13, 7],
"test": [4, 7]
}

remap_dict14_idc_to_expander = {1:"IO0_6", 2:"GND", 3:"IO0_5", 4:"IO0_4", 5:"IO0_3", 6:"IO0_2", 7:"IO0_1", 8:"IO0_0", 
								9:"IO1_7", 10:"NC", 11:"IO1_0", 12:"NC", 13:"IO1_1", 14:"IO1_2"}

remap_dict16_idc_to_expander = {1:"NC", 2:"GND", 3:"IO0_6", 4:"NC", 5:"IO0_5", 6:"IO0_4", 7:"IO0_3", 8:"IO0_2", 
								9:"NC", 10:"IO1_4", 11:"IO1_6", 12:"IO1_5", 13:"NC", 14:"IO1_3", 15:"NC", 16:"NC"}

def remap_panel(panel, pin_map, inverted=False, connector_num_of_pins=None):
    """
    Return a remapped copy of the panel dictionary.

    If inverted=True, converts each remapped IDC pin number using:

        idc_pin_inverted = connector_num_of_pins - idc_pin

    Does not modify the original panel.
    """

    if inverted and connector_num_of_pins is None:
        raise ValueError("connector_num_of_pins must be provided when inverted=True")

    remapped_panel = {}

    for name, pins in panel.items():
        remapped_pins = []

        for pin in pins:
            if inverted:
                if (pin % 2) == 0:
                    idc_pin = pin_map[2 + connector_num_of_pins - pin]
                else:
                    idc_pin = pin_map[1 + connector_num_of_pins - pin]
            else:
                idc_pin = pin_map[pin]

            remapped_pins.append(idc_pin)

        remapped_panel[name] = remapped_pins

    return remapped_panel

pinout_expander = remap_panel(left_panel, remap_dict14_idc_to_expander, inverted=True, connector_num_of_pins=14)

print("Left panel pinout:")
print(pinout_expander)
