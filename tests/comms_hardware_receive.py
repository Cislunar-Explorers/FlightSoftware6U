from communications .satellite_radio import Radio

#Initialize variables, nothing to modify on this end
transmissionCounter = 0
radio = Radio()

while True:
    message = radio.receiveSignal()
    if message is not None:
        print('Transmission #' + transmissionCounter + ' Received: ' + str(message))