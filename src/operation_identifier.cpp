#include "operation_identifier.h"

std::string getPixhawkModeForOperationIdentifier(
    const OperationIdentifier& operation_identifier) {
    switch (operation_identifier) {
        case OperationIdentifier::LAND:
            return PIXHAWK_MODE_LAND;
        default:
            return PIXHAWK_MODE_OFFBOARD;
    }
}

std::string getStringFromOperationIdentifier(const OperationIdentifier& operation_identifier) {
    switch (operation_identifier) {
        case OperationIdentifier::TAKE_OFF:
            return "TAKE_OFF";
        case OperationIdentifier::HOLD:
            return "HOLD";
        case OperationIdentifier::EXPLORE:
            return "EXPLORE";
        case OperationIdentifier::LAND:
            return "LAND";
        case OperationIdentifier::UNDEFINED:
            return "UNDEFINED";
    }
    return "";  // to avoid warning
}
