#! /usr/bin/python3
import os
import sys
import json


ADCs={"name":"ADC"}
INPUTS={"name":"INPUT"}
OUTPUTS={"name":"OUTPUT"}
PPS={"name":"PPS"}
UARTS={"name":"UART"}
SPI={"name":"SPI"}
LEDS={"name":"LED"}
USB={"name":"USB"}
PWM={"name":"PWM"}

roles = [
ADCs,
INPUTS,
OUTPUTS,
PPS,
UARTS,
SPI,
LEDS,
USB,
PWM]

CN7={
    "name": "CN7",
    1 : "PC10",
    2 : "PC11",
    3 : "PC12",
    4 : "PD2",
    5 : "-VDD-",
    6 : "-E5V-",
    7 : "BOOT0",
    8 : "-GND-",
    9 : "-NC-",
    10 : "-NC-",
    11 : "-NC-",
    12 : "-IOREF-",
    13 : "PA13",
    14 : "-RESET-",
    15 : "PA14",
    16 : "-3V3-",
    17 : "PA15",
    18 : "-5V-",
    19 : "-GND-",
    20 : "-GND-",
    21 : "PB7",
    22 : "-GND-",
    23 : "PC13",
    24 : "-VIN-",
    25 : "PC14",
    26 : "-NC-",
    27 : "PC15",
    28 : "PA0",
    29 : "PF0",
    30 : "PA1",
    31 : "PF1",
    32 : "PA4",
    33 : "-VBAT-",
    34 : "PB0",
    35 : "PC2",
    36 : "PC1",
    37 : "PC3",
    38 : "PC0",
}

CN10={
    "name": "CN10",
    1 : "PC9"  ,
    2 : "PC8"  ,
    3 : "PB8"  ,
    4 : "PC6"  ,
    5 : "PB9"  ,
    6 : "PC5"  ,
    7 : "-AVDD-",
    8 : "-U5V-",
    9 : "-GND-",
    10 : "-NC-",
    11 : "PA5" ,
    12 : "PA12",
    13 : "PA6" ,
    14 : "PA11",
    15 : "PA7" ,
    16 : "PB12",
    17 : "PB6" ,
    18 : "PB11",
    19 : "PC7" ,
    20 : "-GND-",
    21 : "PA9" ,
    22 : "PB2" ,
    23 : "PA8" ,
    24 : "PB1" ,
    25 : "PB10",
    26 : "PB15",
    27 : "PB4" ,
    28 : "PB14",
    29 : "PB5" ,
    30 : "PB13",
    31 : "PB3" ,
    32 : "-AGND-",
    33 : "PA10",
    34 : "PC4" ,
    35 : "PA2" ,
    36 : "-NC-",
    37 : "PA3" ,
    38 : "-NC-",
}


connectors = {"CN7" : CN7, "CN10": CN10}


def lookup_io_board(gpio_name):
    for role in roles:
        if gpio_name in role:
            return role


def print_io_board(gpio_name):
    role = lookup_io_board(gpio_name)
    if role:
        print("%s = %s %s" % (gpio_name, role["name"], role[gpio_name]))


def lookup_connector(gpio_name, connector):
    connector_name = connector["name"]
    for p in connector:
        if connector[p] == gpio_name:
            print("%s %u = %s" % (connector_name, p, gpio_name))
            print_io_board(gpio_name)
            return True


def lookup_gpio(connector, pin):
    connector_name = connector["name"]
    gpio_name = connector[pin]
    print("%s %u = %s" % (connector_name, pin, gpio_name))
    print_io_board(gpio_name)
    sys.exit(0)


def print_connect(connector):
    pins = [ pin for pin in connector ]
    pins.remove("name")
    pins.sort()

    for pin in pins:
        gpio = connector[pin]
        if gpio.startswith("P"):
            role = lookup_io_board(gpio)
            if role:
                print("%4s %2u %4s = %s %s" % (connector["name"], pin, gpio, role["name"], role[gpio]))
            else:
                print("%4s %2u %4s = <UNUSED>" % (connector["name"], pin, gpio))
        else:
            print("%4s %2u      = %4s" % (connector["name"], pin, gpio))

def print_help():

    print_connect(CN7)
    print_connect(CN10)

    print("="*72)
    print("Look up GPIO by connector and pin:")
    print("<connector> <pin>")
    print("connector = CN7 or CN10")
    print("pin = number")
    print("="*72)
    print("Look up role and connector and pin:")
    print("<GPIO Name>")
    print("="*72)
    print("Print GPIO roles:")
    print('"roles"')
    print("="*72)



def load_line(subject_map, line):
    parts = line.split(',')
    port = parts[0].strip().replace("{","").replace("GPIO","P")
    pin = parts[1].strip().replace("}","")
    name = parts[2].strip().replace("/", "").replace("\\","")
    name = name.replace("*","").strip()
    name = name.split("=")[0].strip()
    type_info = " ".join(name.split(" ")[1:])
    pins = [ p.strip()[4:] for p in pin.split('|') ]
    for pin in pins:
        subject_map[port + pin] = type_info


def load_uart_line(line):
    parts = line.split(',')
    port = parts[3].strip().replace("GPIO","P")
    pins = [pin.strip()[4:] for pin in parts[4].split('|')]
    name = parts[-1].strip().replace("/", "").replace("\\","")
    name = name.replace("*","").strip()
    name = name.split("=")[0].strip()
    type_info = " ".join(name.split(" ")[1:])
    for pin in pins:
        UARTS[port + pin] = type_info



def main():

    own_dir = os.path.dirname(sys.argv[0])
    with open(own_dir + "/pinmap.h") as f:
        for line in f:
            for role in roles:
                name = role["name"]
                if line.find("/* %s" % name) != -1:
                    if name == "UART":
                        load_uart_line(line)
                    else:
                        load_line(role, line)
                    break

    if len(sys.argv) < 2:
        print_help()
        sys.exit(0)

    if len(sys.argv) == 2:
        gpio_name = sys.argv[1]
        if lookup_connector(gpio_name, CN7) or \
          lookup_connector(gpio_name, CN10):
              sys.exit(0)
        if gpio_name == "roles":
            for role in roles:
                print(role["name"],":")
                keys = list(role.keys())
                keys.remove("name")
                for k in keys:
                    print("\t%5s %s" % (k, role[k]))
            sys.exit(0)
        else:
            print('Not known GPIO or "roles"')
        sys.exit(-1)

    connector = sys.argv[1]
    pin = int(sys.argv[2])

    if connector not in connectors:
        print("%s is not CN7 or CN10" % connector)
        sys.exit(-1)

    lookup_gpio(connectors[connector], pin)



if __name__ == "__main__":
    main()
