/**
 * @file operation_identifier.h
 */

#ifndef OPERATION_IDENTIFIER_H
#define OPERATION_IDENTIFIER_H

#include <map>
#include <string>

const std::string PIXHAWK_MODE_OFFBOARD = "OFFBOARD";
const std::string PIXHAWK_MODE_LAND = "AUTO.LAND";

/**
 * @brief Represents the different operations.
 */
enum class OperationIdentifier {
    // Null is here to make the bredth first search possible in operation graph
    TAKE_OFF,
    HOLD,
    EXPLORE,
    LAND,
    UNDEFINED
};

/**
 * @brief Retrieves the Pixhawk mode for a given @p operation_identifier.
 *
 * @param operation_identifier The operation identifier to get the Pixhawk mode for.
 *
 * @return The Pixhawk mode.
 */
std::string getPixhawkModeForOperationIdentifier(const OperationIdentifier& operation_identifier);

/**
 * @brief Get a string from the @p operation_identifier.
 *
 * @param operation_identifier The operation identifier.
 *
 * @return The operation identifier represented in a string.
 */
std::string getStringFromOperationIdentifier(const OperationIdentifier& operation_identifier);

#endif
