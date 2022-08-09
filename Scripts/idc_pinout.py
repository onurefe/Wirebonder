# These are the noted values from my numbering system.
left_panel = {"tail_plus": [13, 8],
"tail_minus": [14, 8],
"step_minus": [2, 8],
"step_plus": [3, 8],
"loop_minus": [14, 7],
"loop_plus": [2, 7],
"search_minus": [14, 9],
"search_plus": [2, 9],
"up": [4, 7],
"right": [12, 8],
"down": [13, 7],
"left": [3, 9],
"minus": [3, 7],
"plus": [12, 7],
"enter": [4, 9],
"load": [4,8],
"save": [11, 7],
"add": [11, 9],
"esc": [12, 9],
"manual": [13, 9],
"led_anode":[10],
"led_cathode":[1]}

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

remap_dict14 = {1:2, 2:4, 3:6, 4:8, 5:10, 6:12, 7:14, 8:13, 9:11, 10:9, 11:7, 12:5, 13:3, 14:1}

remap_dict16 = {1:2, 2:4, 3:6, 4:8, 5:10, 6:12, 7:14, 8:16, 9:15, 10:13, 11:11, 12:9, 13:7, 14:5, 15:3, 16:1}

def remap(panel, remap_dict):
	for key in panel.keys():
		for idx, pin in enumerate(panel[key]):
			panel[key][idx] = remap_dict[pin]
	
	return panel

print("Left panel pinout:")
print(remap(left_panel, remap_dict14))

print("Right panel pinout:")
print(remap(right_panel, remap_dict16))
