#include "dataPID.h"

dataPID::dataPID(std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);

    Kp_Long_Line = new DataAbstract<float>("Kp_Long_Line", name, 0.0421);
    Kd_Long_Line = new DataAbstract<float>("Kd_Long_Line", name, 0.0978);

    Kp_Medium_Line = new DataAbstract<float>("Kp_Medium_Line", name, 0.0421);
    Kd_Medium_Line = new DataAbstract<float>("Kd_Medium_Line", name, 0.0978);

    Kp_Short_Line = new DataAbstract<float>("Kp_Short_Line", name, 0.0421);
    Kd_Short_Line = new DataAbstract<float>("Kd_Short_Line", name, 0.0978);

    Kp_Long_Curve = new DataAbstract<float>("Kp_Long_Curve", name, 0.0421);
    Kd_Long_Curve = new DataAbstract<float>("Kd_Long_Curve", name, 0.0978);

    Kp_Medium_Curve = new DataAbstract<float>("Kp_Medium_Curve", name, 0.0421);
    Kd_Medium_Curve = new DataAbstract<float>("Kd_Medium_Curve", name, 0.0978);

    Kp_Short_Curve = new DataAbstract<float>("Kp_Short_Curve", name, 0.0421);
    Kd_Short_Curve = new DataAbstract<float>("Kd_Short_Curve", name, 0.0978);

    Kp_Zigzag = new DataAbstract<float>("Kp_zigzag", name, 0.0421);
    Kd_Zigzag = new DataAbstract<float>("Kd_zigzag", name, 0.0978);

    Kp_Default = new DataAbstract<float>("Kp_Default", name, 0.0421);
    Kd_Default = new DataAbstract<float>("Kd_Default", name, 0.0978);

    Kp_Tunning = new DataAbstract<float>("Kp_Tunning", name, 0.0421);
    Kd_Tunning = new DataAbstract<float>("Kd_Tunning", name, 0.0978);
}

DataAbstract<float> *dataPID::Kp(TrackState state)
{
    if(state == LONG_LINE){
        return Kp_Long_Line;
    }
    else if(state == MEDIUM_LINE){
        return Kp_Medium_Line;
    }
    else if(state == SHORT_LINE){
        return Kp_Short_Line;
    }
    else if(state == LONG_CURVE){
        return Kp_Long_Curve;
    }
    else if(state == MEDIUM_CURVE){
        return Kp_Medium_Curve;
    }
    else if(state == SHORT_CURVE){
        return Kp_Short_Curve;
    }
    else if(state == ZIGZAG){
        return Kp_Zigzag;
    }
    else if(state == TUNNING){
        return Kp_Tunning;
    }
    else{
        return Kp_Default;
    }
    ESP_LOGE(tag, "Estado do Robô ou Objeto PID inválido para esse método: %s:%d para obter o Kp do PID, retornando valor null.", name.c_str(),state);
    return nullptr;
}

DataAbstract<float> *dataPID::Kd(TrackState state)
{
    if(state == LONG_LINE){
        return Kd_Long_Line;
    }
    else if(state == MEDIUM_LINE){
        return Kd_Medium_Line;
    }
    else if(state == SHORT_LINE){
        return Kd_Short_Line;
    }
    else if(state == LONG_CURVE){
        return Kd_Long_Curve;
    }
    else if(state == MEDIUM_CURVE){
        return Kd_Medium_Curve;
    }
    else if(state == SHORT_CURVE){
        return Kd_Short_Curve;
    }
    else if(state == ZIGZAG){
        return Kd_Zigzag;
    }
    else if(state == TUNNING){
        return Kd_Tunning;
    }
    else{
        return Kd_Default;
    }
    ESP_LOGE(tag, "Estado do Robô ou Objeto PID inválido para esse método: %s:%d para obter o Kd do PID, retornando valor null.", name.c_str(),state);
    return nullptr;
}