#include "feet.h"
#include "vision.h"
#include "robot.h"
#include "commandLine.h"
#include "config/configRegistry.h"
#include "motorbus/motorbus.h"

#include <iostream>


/*------------------------------------------------------------------------------------------------*/

class FeetCmdLineCallback : public CommandLineInterface {
public:
	virtual bool commandLineCallback(CommandLine &cmdLine) {
		std::string cmd = cmdLine.getCommand(0);

		if (cmd == "kickpos") {
			ERROR("TEMPORARLIY DISABLED");
			robot.getConfig().save();

			return true;
/*
			int kickPosRightX = 0, kickPosLeftX = 0, kickPosRightY = 0, kickPosLeftY = 0;

			while (true) {
				printf("\n\n\n");
				printf("Calibrating left foot kickposition\n");
				printf("\n");
				printf("Please make sure the vision sees the ball and then place the ball\n");
				printf("in front of the \033[31mLEFT\033[m foot.\n");
				printf("\n");
				printf("Press RETURN when the ball has been placed.\n\n\n");

				do {
					PositionRelative p        = wm.getBall()->getPosRelFromVision();
					robottime_t      lastSeen = wm.getBall()->getLastSeenFromVision();

					printf("\033[1A");
					if (lastSeen + 250 < getCurrentTime())
						printf("  'LEFT' Ball position: \033[31mBall not seen\033[0m        \n");
					else {
						kickPosLeftX = p.getX();
						kickPosLeftY = p.getY();
						printf("  'LEFT' Ball position: ( %- 2d, %- 2d)          \n", p.getX(), p.getY());
					}

					fflush(stdout);
				} while (getKey(250) == 0);

				printf("\n\n");
				printf("Please place the ball in front of the \033[31mRIGHT\033[m foot.\n");
				printf("\n");
				printf("Press RETURN when the ball has been placed.\n\n\n");

				do {
					PositionRelative p        = wm.getBall()->getPosRelFromVision();
					robottime_t      lastSeen = wm.getBall()->getLastSeenFromVision();

					printf("\033[1A");
					if (lastSeen + 250 < getCurrentTime())
						printf(" 'RIGHT' Ball position: \033[31mBall not seen\033[0m        \n");
					else {
						kickPosRightX = p.getX();
						kickPosRightY = p.getY();
						printf(" 'RIGHT' Ball position: ( %- 2d, %- 2d)          \n", p.getX(), p.getY());
					}

					fflush(stdout);
				} while (getKey(250) == 0);

				if (kickPosLeftX >= kickPosRightX) {
					printf("\n\n\n");
					printf("\033[31mLeft kick position seems to be farther to the right than the\033[0m\n");
					printf("\033[31mright kick position. This does not sound right, please try again!\033[0m\n");

				} else
					break; // done (presumably)
			}

			Feet::getInstance().setKickPositionLeft(kickPosLeftX, kickPosLeftY);
			Feet::getInstance().setKickPositionRight(kickPosRightX, kickPosRightY);

			printf("Saving ... ");
			fflush(stdout);
			robot.getConfig().save();
			printf("done\n\n");
			return true;
*/
		}

		return false;
	}
};

static FeetCmdLineCallback feetCmdLineCallback;
REGISTER_COMMAND("kickpos", "Calibrate kick positions", false, true, false, &feetCmdLineCallback);

static const bool feetLeftOption   = ConfigRegistry::getInstance().registerOption("feet.left",             80, "Left side in image coordinates of feet area");
static const bool feetRightOption  = ConfigRegistry::getInstance().registerOption("feet.right",           560, "Right side in image coordinates of feet area");
static const bool feetTopOption    = ConfigRegistry::getInstance().registerOption("feet.top",             400, "Top side in image coordinates of feet area");
static const bool feetBottomOption = ConfigRegistry::getInstance().registerOption("feet.bottom",          480, "Bottom side in image coordinates of feet area");

/*------------------------------------------------------------------------------------------------*/

Feet::Feet() {
	comm.registerOperationCallback(this, OP_GETFEETSPACE, 0, 0);
	comm.registerOperationCallback(this, OP_SETFEETSPACE, 8, 8);

	Events::getInstance().registerForEvent(EVT_CONFIGURATION_LOADED, this);
	Events::getInstance().registerForEvent(EVT_BEFORE_CONFIG_SAVE, this);

	// as we may be constructed AFTER the configuration is loaded,
	// let's trigger a configuration initialization now
	eventCallback(EVT_CONFIGURATION_LOADED, &robot.getConfig());
}


/*------------------------------------------------------------------------------------------------*/

Feet::~Feet() {
	comm.unregisterOperationCallback(this, OP_GETFEETSPACE);
	comm.unregisterOperationCallback(this, OP_SETFEETSPACE);

	Events::getInstance().unregisterForEvent(EVT_CONFIGURATION_LOADED, this);
	Events::getInstance().unregisterForEvent(EVT_BEFORE_CONFIG_SAVE, this);
}


/*------------------------------------------------------------------------------------------------*/

/**
 *
 * @param evtType
 * @param
 */
void Feet::eventCallback(EventType evtType, void*) {
	Config &config = robot.getConfig();

	// if configuration is not yet loaded, abort
	if (&config == 0)
		return;

	if (evtType == EVT_CONFIGURATION_LOADED) {
		feetSpace.rectangle.x      = config.getIntValue("feet.left",   feetSpace.rectangle.x);
		feetSpace.rectangle.width  = config.getIntValue("feet.right",  feetSpace.rectangle.x + feetSpace.rectangle.width)  - feetSpace.rectangle.x;
		feetSpace.rectangle.y      = config.getIntValue("feet.top",    feetSpace.rectangle.y);
		feetSpace.rectangle.height = config.getIntValue("feet.bottom", feetSpace.rectangle.y + feetSpace.rectangle.height) - feetSpace.rectangle.y;

	} else if (evtType == EVT_BEFORE_CONFIG_SAVE) {
		config.setValue("feet.left",             feetSpace.rectangle.x);
		config.setValue("feet.right",            feetSpace.rectangle.width + feetSpace.rectangle.x);
		config.setValue("feet.top",              feetSpace.rectangle.y);
		config.setValue("feet.bottom",           feetSpace.rectangle.height +feetSpace.rectangle.y);
	}
}


/*------------------------------------------------------------------------------------------------*/

/**
 * Get the current feet space
 * @param left
 * @param top
 * @param right
 * @param bottom
 */
void Feet::getFeetSpace(int16_t &left, int16_t &top, int16_t &right, int16_t &bottom) {
	left   = feetSpace.rectangle.x;
	right  = feetSpace.rectangle.x + feetSpace.rectangle.width;
	top    = feetSpace.rectangle.y;
	bottom = feetSpace.rectangle.y + feetSpace.rectangle.height;
}


/*------------------------------------------------------------------------------------------------*/

/** Handle feet related operations
 **
 ** @param operation       Operation
 ** @param flags           Operation flags
 ** @param data            Received payload
 ** @param dataLen         Number of bytes received in payload
 ** @param remoteAddress   Information about who sent this operation
 **
 ** @return true iff operation was handled successfully
 */

bool Feet::operationCallback(
	OPERATION operation,
	uint8_t   flags,
	uint8_t  *data,
	uint16_t  dataLen,
	struct sockaddr_in *remoteAddress)
{
	if (operation == OP_GETFEETSPACE) {
		INFO("Sending feed space");
		int16_t response[4];
		getFeetSpace(response[0], response[1], response[2], response[3]);
		for (uint8_t i = 0; i < 4; i++)
			response[i] = htons(response[i]);

		comm.sendMessage(OP_GETFEETSPACE, FLAG_IS_ANSWER, (uint8_t*)response, sizeof response, remoteAddress);
	} else if (operation == OP_SETFEETSPACE) {
		INFO("Receiving feed space");
		int16_t *values = (int16_t*)data;

		feetSpace.rectangle.x   = ntohs(values[0]);
		feetSpace.rectangle.y    = ntohs(values[1]);
		feetSpace.rectangle.width  = ntohs(values[2]) - feetSpace.rectangle.x;
		feetSpace.rectangle.height = ntohs(values[3]) - feetSpace.rectangle.y;
	} else
		return false;

	return true;
}
