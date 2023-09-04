#ifndef RELIABLE_LORA_H
    #define RELIABLE_LORA_H

    #ifndef LoRa_E220_h
        #include <LoRa_E220.h> 
    #endif
    unsigned long int timeOutSYNACK = 5000;
    unsigned long int timeOutACK = 5000;
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
        Serial.println("SYN RECEBIDO");
        lora.setMode(MODE_0_NORMAL);
        sendSYNACK(lora, channel, 1);
        Serial.println("SYNACK ENVIADO");
        while(waitACK(lora, &pck) == 0){
            if(millis() - startTime >= timeOutHandshake){
                return -1;}
            else
                sendSYNACK(lora, channel, 1);
        }
        Serial.println("ACK RECEBIDO");
        return 1;
    }

    int senderHandshake(LoRa_E220 lora, byte OP){
        struct Packet<byte> pck;
        unsigned long startTime = millis();
        sendSYN(lora,channel, OP);
        Serial.println("SYN ENVIADO");
        while(waitSYNACK(lora, &pck) == 0){
            if(millis() - startTime >= timeOutHandshake){
                return 0;
            }
            else{
                sendSYN(lora, channel, OP);
                Serial.println("SYN ENVIADO");
            }
        }
        Serial.println("SYNACK RECEBIDO");
        sendACK(lora, channel, OP);
        Serial.println("ACK ENVIADO");
        return 1;
    }
#endif
