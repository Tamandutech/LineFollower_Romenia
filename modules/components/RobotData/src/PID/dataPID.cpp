#include "dataPID.h"

dataPID::dataPID(std::string name)
{// Objeto utilizado para guardar contantes do cálculo PID
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    //ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);

    dataManager = dataManager->getInstance();

    output = new DataAbstract<float>("output", name, 0);
    dataManager->registerRuntimeData(output);
    setpoint = new DataAbstract<float>("Setpoint", name, 0);
    dataManager->registerRuntimeData(setpoint);
    erro = new DataAbstract<float>("erro", name, 0);
    dataManager->registerRuntimeData(erro);
    erroquad = new DataAbstract<float>("erroquad", name, 0);
    dataManager->registerRuntimeData(erroquad);

    Kp_Linear = new DataAbstract<float>("Kp_Linear", name, 5.43);
    dataManager->registerParamData(Kp_Linear);
    Kd_Linear = new DataAbstract<float>("Kd_Linear", name, 5.8);
    dataManager->registerParamData(Kd_Linear);
    Ki_Linear = new DataAbstract<float>("Ki_Linear", name, 0);
    dataManager->registerParamData(Ki_Linear);

    Kp_Long_Line = new DataAbstract<float>("Kp_Long_Line", name, 5.43);
    dataManager->registerParamData(Kp_Long_Line);
    Kd_Long_Line = new DataAbstract<float>("Kd_Long_Line", name, 5.8);
    dataManager->registerParamData(Kd_Long_Line);

    Kp_Medium_Line = new DataAbstract<float>("Kp_Medium_Line", name, 5.43);
    dataManager->registerParamData(Kp_Medium_Line);
    Kd_Medium_Line = new DataAbstract<float>("Kd_Medium_Line", name, 5.8);
    dataManager->registerParamData(Kd_Medium_Line);

    Kp_Short_Line = new DataAbstract<float>("Kp_Short_Line", name, 5.43);
    dataManager->registerParamData(Kp_Short_Line);
    Kd_Short_Line = new DataAbstract<float>("Kd_Short_Line", name, 5.8);
    dataManager->registerParamData(Kd_Short_Line);

    Kp_Long_Curve = new DataAbstract<float>("Kp_Long_Curve", name, 5.43);
    dataManager->registerParamData(Kp_Long_Curve);
    Kd_Long_Curve = new DataAbstract<float>("Kd_Long_Curve", name, 5.8);
    dataManager->registerParamData(Kd_Long_Curve);

    Kp_Medium_Curve = new DataAbstract<float>("Kp_Medium_Curve", name, 5.43);
    dataManager->registerParamData(Kp_Medium_Curve);
    Kd_Medium_Curve = new DataAbstract<float>("Kd_Medium_Curve", name, 5.8);
    dataManager->registerParamData(Kd_Medium_Curve);

    Kp_Short_Curve = new DataAbstract<float>("Kp_Short_Curve", name, 5.43);
    dataManager->registerParamData(Kp_Short_Curve);
    Kd_Short_Curve = new DataAbstract<float>("Kd_Short_Curve", name, 5.8);
    dataManager->registerParamData(Kd_Short_Curve);

    Kp_Zigzag = new DataAbstract<float>("Kp_zigzag", name, 5.43);
    dataManager->registerParamData(Kp_Zigzag);
    Kd_Zigzag = new DataAbstract<float>("Kd_zigzag", name, 5.8);
    dataManager->registerParamData(Kd_Zigzag);

    Kp_Default = new DataAbstract<float>("Kp_Default", name, 5.43);
    dataManager->registerParamData(Kp_Default);
    Kd_Default = new DataAbstract<float>("Kd_Default", name, 5.8);
    dataManager->registerParamData(Kd_Default);

    Kp_Tunning = new DataAbstract<float>("Kp_Tunning", name, 5.43);
    dataManager->registerParamData(Kp_Tunning);
    Kd_Tunning = new DataAbstract<float>("Kd_Tunning", name, 5.8);
    dataManager->registerParamData(Kd_Tunning);
}

DataAbstract<float> *dataPID::Kp(TrackState state)
{// Retorna o valor de K_P para cada trecho da pista
    switch(state)
    {
        case LONG_LINE:
            return Kp_Long_Line;
            break;
        case MEDIUM_LINE:
            return Kp_Medium_Line;
            break;
        case SHORT_LINE:
            return Kp_Short_Line;
            break;
        case LONG_CURVE:
            return Kp_Long_Curve;
            break;
        case MEDIUM_CURVE:
            return Kp_Medium_Curve;
            break;
        case SHORT_CURVE:
            return Kp_Short_Curve;
            break;
        case ZIGZAG:
            return Kp_Zigzag;
            break;
        case TUNNING:
            return Kp_Tunning;
            break;
        default:
            return Kp_Default;
            break;
    }

    //ESP_LOGE(tag, "Estado do Robô ou Objeto PID inválido para esse método: %s:%d para obter o Kp do PID, retornando valor null.", name.c_str(),state);
    return nullptr;
}

DataAbstract<float> *dataPID::Kd(TrackState state)
{// Retorna o valor de K_D para cada trecho da pista
    switch(state)
    {
        case LONG_LINE:
            return Kd_Long_Line;
            break;
        case MEDIUM_LINE:
            return Kd_Medium_Line;
            break;
        case SHORT_LINE:
            return Kd_Short_Line;
            break;
        case LONG_CURVE:
            return Kd_Long_Curve;
            break;
        case MEDIUM_CURVE:
            return Kd_Medium_Curve;
            break;
        case SHORT_CURVE:
            return Kd_Short_Curve;
            break;
        case ZIGZAG:
            return Kd_Zigzag;
            break;
        case TUNNING:
            return Kd_Tunning;
            break;
        default:
            return Kd_Default;
            break;
    }

    //ESP_LOGE(tag, "Estado do Robô ou Objeto PID inválido para esse método: %s:%d para obter o Kd do PID, retornando valor null.", name.c_str(),state);
    return nullptr;
}