#ifndef RELIABLE_LORA_H
    #define RELIABLE_LORA_H

    #ifndef LoRa_E220_h
        #include <LoRa_E220.h> 
    #endif
    unsigned long int timeOutSYNACK = 3000;
    unsigned long int timeOutACK = 3000;
    unsigned long int timeToRetry = 1000;
    byte senderAddr[2] = {0,0};
    byte receptorAddr[2] = {0,0};
    byte channel = 0;

    template<typename T>
    struct Packet{
        T data;
        u_int16_t Checksum;
        byte OP;
        byte senderAddH;
        byte senderAddL;
        bool ACK = false;
        bool SYN = false;
    };

    void getADDR(LoRa_E220 lora, byte* addr){
        ResponseStructContainer c;
        c = lora.getConfiguration();
        Configuration configuration = *(Configuration*) c.data;
        addr[0] = configuration.ADDH;
        addr[1] = configuration.ADDL;
    }

    byte sendSYN(LoRa_E220 lora, byte channel, byte OP){
        struct Packet<byte> pck;
        byte addr[2];
        pck.senderAddH = senderAddr[0];
        pck.senderAddL = senderAddr[1];
        pck.data = 0;
        pck.SYN = true;
        pck.OP = OP;
        ResponseStatus response = lora.sendFixedMessage(receptorAddr[0],receptorAddr[1],channel,&pck,sizeof(Packet<byte>));
        return response.code;

    }

    byte sendACK(LoRa_E220 lora, byte channel, byte OP){
        struct Packet<byte> pck;
        byte addr[2];
        pck.senderAddH = senderAddr[0];
        pck.senderAddL = senderAddr[1];
        pck.data = 0;
        pck.ACK = true;
        pck.OP = OP;
        ResponseStatus response = lora.sendFixedMessage(receptorAddr[0],receptorAddr[1],channel,&pck,sizeof(Packet<byte>));
        return response.code;
    }

    byte sendSYNACK(LoRa_E220 lora, byte channel, byte OP){
        struct Packet<byte> pck;
        byte addr[2];
        pck.senderAddH = senderAddr[0];
        pck.senderAddL = senderAddr[1];
        pck.data = 0;
        pck.SYN = true;
        pck.ACK = true;
        pck.OP = OP;
        ResponseStatus response = lora.sendFixedMessage(receptorAddr[0],receptorAddr[1],channel,&pck,sizeof(Packet<byte>));
        return response.code;
    }

   byte waitSYN(LoRa_E220 lora, struct Packet<byte>* pck, byte* operation){
        if (lora.available()  > 1){
            ResponseStructContainer rsc = lora.receiveMessageRSSI(sizeof(Packet<byte>));
            *pck = *(Packet<byte>*) rsc.data;
            rsc.close();
            if(pck->SYN == true){
                *operation = pck->OP;
                receptorAddr[0] = pck->senderAddH;
                receptorAddr[1] = pck->senderAddL;
                return 1;
            }
        }
        return 0;
    }

    byte waitSYNACK(LoRa_E220 lora, struct Packet<byte>* pck){
        unsigned long startTime = millis();
        while (millis() - startTime < timeOutSYNACK){

            if (lora.available()  > 1){
                ResponseStructContainer rsc = lora.receiveMessageRSSI(sizeof(Packet<byte>));
                *pck = *(Packet<byte>*) rsc.data;
                rsc.close();
                if(pck->SYN == true && pck->ACK == true){
                    return 1;
                }
            }
        }
        return 0;
    }

    byte waitACK(LoRa_E220 lora, struct Packet<byte>* pck){
        unsigned long startTime = millis();
        while (millis() - startTime < timeOutACK){
            if (lora.available()  > 1){
                ResponseStructContainer rsc = lora.receiveMessageRSSI(sizeof(Packet<byte>));
                *pck = *(Packet<byte>*) rsc.data;
                rsc.close();
                if(pck->ACK == true){
                    return 1;

                }
            }
        }
        return 0;
    }
    int receptorHandshake(LoRa_E220 lora, byte* OP){
        struct Packet<byte> pck;
        unsigned long startTime = millis();
        while(waitSYN(lora, &pck, OP) == 0){
            if(millis() - startTime >= timeOutHandshake){return 0;}
        }
        lora.setMode(MODE_0_NORMAL);
        sendSYNACK(lora, channel, 1);
        unsigned long retryTime = millis() + timeToRetry;
        while(waitACK(lora, &pck) == 0){
            if(millis() - startTime >= timeOutHandshake){
                return -1;}
            else if(millis() >= retryTime){
                retryTime = millis() + timeToRetry;
                sendSYNACK(lora, channel, 1);
            }
        }
        return 1;
    }

    int senderHandshake(LoRa_E220 lora, byte OP){
        struct Packet<byte> pck;
        unsigned long startTime = millis();
        sendSYN(lora,channel, OP);
        unsigned long retryTime = millis() + timeToRetry;
        while(waitSYNACK(lora, &pck) == 0){
            if(millis() - startTime >= timeOutHandshake){
                return 0;
            }
            else if(millis() >= retryTime){
                sendSYN(lora, channel, OP);
                retryTime = millis() + timeToRetry;
            }
        }
        sendACK(lora, channel, OP);
        return 1;
    }
#endif
