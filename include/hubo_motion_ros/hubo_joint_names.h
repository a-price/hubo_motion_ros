/**
 * \file hubo_joint_names.h
 * \brief Contains std::strings for each of Hubo's joints, indexed according to <hubo.h>
 *
 * \author Andrew Price
 */
#ifndef HUBO_JOINT_NAMES_H
#define HUBO_JOINT_NAMES_H

#include <string>
#include <map>

#include <hubo.h>


/**
 * \var HUBO_JOINT_NAMES
 * \brief Contains std::strings for each of Hubo's joints, indexed according to <hubo.h>
 * NB: The size of the array must be equal to HUBO_JOINT_COUNT as defined in <hubo.h>
 */
const std::string HUBO_JOINT_NAMES[] = 
{
	"WST","NKY","NK1","NK2",
	"LSP","LSR","LSY","LEB","LWY","LWR","LWP",
	"RSP","RSR","RSY","REB","RWY","RWR","RWP","",
	"LHY","LHR","LHP","LKN","LAP","LAR","",
	"RHY","RHR","RHP","RKN","RAP","RAR",
	"RF1","RF2","RF3","RF4","RF5",
	"LF1","LF2","LF3","LF4","LF5",
};

/**
 * \var HUBO_URDF_JOINT_NAMES
 * \brief Contains std::strings for each of Hubo's joints in the URDF model, indexed according to <hubo.h>
 * NB: The size of the array must be equal to HUBO_JOINT_COUNT as defined in <hubo.h>
 */
const std::string HUBO_URDF_JOINT_NAMES[] = 
{
	"HPY","NKY","NK1","NK2", //HNR, HNP ?
	"LSP","LSR","LSY","LEP","LWY","LWR","LWP",
	"RSP","RSR","RSY","REP","RWY","RWR","RWP","",
	"LHY","LHR","LHP","LKP","LAP","LAR","",
	"RHY","RHR","RHP","RKP","RAP","RAR",
	"rightThumbKnuckle1","rightIndexKnuckle1","rightMiddleKnuckle1","rightRingKnuckle1","rightPinkyKnuckle1",
	"leftThumbKnuckle1","leftIndexKnuckle1","leftMiddleKnuckle1","leftRingKnuckle1","leftPinkyKnuckle1",
};

/**
 * \var HUBO_JOINT_NAME_TO_INDEX
 * \brief Maps std::strings for each of Hubo's joints to their indices according to <hubo.h>
 * NB: Requires C++11 to compile
 */
const std::map<std::string, int> HUBO_JOINT_NAME_TO_INDEX = {
	{"HPY",WST},
	{"WST",WST},{"NKY",NKY},{"NK1",NK1},{"NK2",NK2},
	{"LSP",LSP},{"LSR",LSR},{"LSY",LSY},{"LEB",LEB},{"LWY",LWY},{"LWR",LWR},{"LWP",LWP},
	{"RSP",RSP},{"RSR",RSR},{"RSY",RSY},{"REB",REB},{"RWY",RWY},{"RWR",RWR},{"RWP",RWP},
	{"LHY",LHY},{"LHR",LHR},{"LHP",LHP},{"LKN",LKN},{"LAP",LAP},{"LAR",LAR},
	{"RHY",RHY},{"RHR",RHR},{"RHP",RHP},{"RKN",RKN},{"RAP",RAP},{"RAR",RAR},
	{"RF1",RF1},{"RF2",RF2},{"RF3",RF3},{"RF4",RF4},{"RF5",RF5},
	{"LF1",LF1},{"LF2",LF2},{"LF3",LF3},{"LF4",LF4},{"LF5",LF5}
};

/**
 * \var HUBO_JOINT_NAME_TO_LIMB_POSITION
 * \brief Maps std::strings for each of Hubo's joints to their place in the limb's chain according to <hubo.h>
 * NB: Requires C++11 to compile
 */
const std::map<std::string, unsigned> HUBO_JOINT_NAME_TO_LIMB_POSITION = {
	{"LSP",0},{"LSR",1},{"LSY",2},{"LEB",3},{"LEP",3},{"LWY",4},{"LWR",5},{"LWP",5},
	{"RSP",0},{"RSR",1},{"RSY",2},{"REB",3},{"REP",3},{"RWY",4},{"RWR",5},{"RWP",5},
	{"LHY",0},{"LHR",1},{"LHP",2},{"LKN",3},{"LAP",4},{"LAR",5},
	{"RHY",0},{"RHR",1},{"RHP",2},{"RKN",3},{"RAP",4},{"RAR",5}
};

/**
 * \var HUBO_JOINT_INDEX_TO_LIMB_POSITION
 * \brief Maps indices for each of Hubo's joints to their place in the limb's chain according to <hubo.h>
 * NB: Requires C++11 to compile
 */
const unsigned HUBO_JOINT_INDEX_TO_LIMB_POSITION[] = {
	0,0,0,0,
	0,1,2,3,4,5,5,
	0,1,2,3,4,5,5,0,
	0,1,2,3,4,5,0,
	0,1,2,3,4,5,
	0,0,0,0,0,
	0,0,0,0,0
};

/**
 * \var HUBO_URDF_FINGER_NAMES
 * \brief Lists the finger names for Hubo's hand for hubo_urdf
 */
const std::string HUBO_URDF_FINGER_NAMES[] =
{
	"Thumb","Index","Middle","Ring","Pinky"
};

/**
 * \var HUBO_URDF_FINGER_LINK_NAMES
 * \brief Lists the finger link names for Hubo's hand for hubo_urdf
 */
const std::string HUBO_URDF_FINGER_LINK_NAMES[] =
{
	"Proximal","Medial","Distal"
};
#endif //HUBO_JOINT_NAMES_H
