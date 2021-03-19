from communications.satellite_radio import Radio
from drivers.gom import Gomspace
import utils.parameters as params
from time import sleep

radio = Radio()
gom = Gomspace()
electrolyzing = False

interval = 3 # Number of seconds between each transmission
repetitions = 5 # Number of total times you want to transmit the pattern

downlink = bytes(input("Enter signal you want to downlink (e.g '\\x08\\x07\\x00\\x00)':"), encoding='utf-8')
downlink = downlink.decode('unicode-escape').encode('ISO-8859-1')
#–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
# Enter Transmit Mode
       
#Stop electrolyzing
if gom.is_electrolyzing():
    electrolyzing = True
    gom.set_electrolysis(False)
    print('Electrolyzers turned off')

#Set RF receiving side to low
gom.rf_receiving_switch(receive=False)
print('RF Receiving set to low')

#Turn off LNA
gom.lna(False)
print('LNA turned off')

#Set RF transmitting side to high
gom.rf_transmitting_switch(receive=False)
print('RF transmitting set to high')

#Turn on power amplifier
gom.set_PA(on=True)
print('Power amplifier turned on')
#–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
#Downlink
radio.transmit(downlink)
for i in range(1, repetitions+1):
    radio.transmit(downlink)
    print('Transmitted: ' + str(i))
    sleep(interval)
#–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
#Enter Receiving Mode

#Turn off power amplifier
gom.set_PA(on=False)
print('Power amplifier turned off')

#Set RF transmitting side to low
gom.rf_transmitting_switch(receive=True)
print('RF transmitting set to low')

#Turn on LNA
gom.lna(True)
print('LNA turned on')

#Set RF receiving side to high
gom.rf_receiving_switch(receive=True)
print('RF receiving set to high')

#Resume electrolysis if we paused it to transmit
if electrolyzing:
    gom.set_electrolysis(True,delay = params.DEFAULT_ELECTROLYSIS_DELAY)
    print('Electrolysis Resumed with Delay ' + str(params.DEFAULT_ELECTROLYSIS_DELAY))
