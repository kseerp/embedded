import os

gpio_path = '/sys/class/gpio/'
export_file_path = '/sys/class/gpio/export'
unexport_file_path = '/sys/class/gpio/unexport'

# Get the list of folders in /sys/class/gpio/
folders = os.listdir(gpio_path)
for folder in folders:
        folder_path = os.path.join(gpio_path, folder)

        label_file = os.path.join(folder_path, 'label')
        if os.path.isfile(label_file):
                with open(label_file, 'r') as file:
                        label_content = file.read().strip()

                        if 'adg1414' in label_content:
                                base_file = os.path.join(folder_path, 'base')
                                if os.path.isfile(base_file):
                                        with open(base_file, 'r') as file:
                                                base_value = file.read().strip()

def write_to_export(base, ngpio):
	base = int(base)
	ngpio = int(ngpio)

	for i in range(ngpio):
		os.system(f'echo {base + i} > {export_file_path}')

def write_to_unexport(base, ngpio):
	base = int(base)
	ngpio = int(ngpio)

	for i in range(ngpio):
		os.system(f'echo {base + i} > {unexport_file_path}')

def attenuation_ch1(val):
	val = int(val)

	if val < 0 or val > 31:
		raise ValueError('Attenuation value must be between 0 and 31')

	att = ~(val & 0x1F)

	for i in range(5):
		bit = (att >> i) & 1
		os.system(f'echo {bit} > /sys/class/gpio/gpio{int(base_value) + i}/value')

def attenuation_ch2(val):
	val = int(val)

	if val < 0 or val > 31:
		raise ValueError('Attenuation value must be between 0 and 31')

	att = ~(val & 0x1F)

	for i in range(5):
		bit = (att >> i) & 1
		os.system(f'echo {bit} > /sys/class/gpio/gpio{int(base_value) + 5 + i}/value')

def vdd(val):
	val = int(val)

	if val < 0 or val > 15:
		raise ValueError('VDD value must be between 0 and 15')

	for i in range(4):
		bit = (val >> i) & 1
		os.system(f'echo {bit} > /sys/class/gpio/gpio{int(base_value) + 10 + i}/value')

def rf_ch1(mode):
	mode = int(mode)

	if mode < 1 or mode > 2:
		raise ValueError('RF channel mode must be 1 or 2')

	for i in range(2):
		bit = (mode >> i) & 1
		os.system(f'echo {bit} > /sys/class/gpio/gpio{int(base_value) + 16 + i}/value')

def rf_ch2(mode):
	mode = int(mode)

	if mode < 1 or mode > 2:
		raise ValueError('RF channel mode must be 1 or 2')

	for i in range(2):
		bit = (mode >> i) & 1
		os.system(f'echo {bit} > /sys/class/gpio/gpio{int(base_value) + 18 + i}/value')


# write_to_unexport(base_value, 24)
# attenuation_ch1(31)
# attenuation_ch2(31)
# vdd(15)
# rf_ch1(1)
# rf_ch2(1)
