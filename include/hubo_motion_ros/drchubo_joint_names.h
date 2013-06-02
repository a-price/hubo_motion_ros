/**
 * \file drchubo_joint_names.h
 * \brief Contains std::strings for each of Hubo's joints, indexed according to <hubo.h>
 *
 * \author Andrew Price
 */
#ifndef DRCHUBO_JOINT_NAMES_H
#define DRCHUBO_JOINT_NAMES_H

#include <string>
#include <map>

#include <hubo.h>


/**
 * \var DRCHUBO_JOINT_NAMES
 * \brief Contains std::strings for each of Hubo's joints, indexed according to <hubo.h>
 * NB: The size of the array must be equal to HUBO_JOINT_COUNT as defined in <hubo.h>
 */
const std::string DRCHUBO_JOINT_NAMES[] =
{
	"TSY","NKY","NKP","",
	"LSP","LSR","LSY","LEB","LWY","LWR","LWP",
	"RSP","RSR","RSY","REB","RWY","RWR","RWP","",
	"LHY","LHR","LHP","LKN","LAP","LAR","",
	"RHY","RHR","RHP","RKN","RAP","RAR",
	"RF1","RF2","RF3","","RF5",
	"LF1","LF2","LF3","","LF5",
};

/**
 * \var DRCHUBO_URDF_JOINT_NAMES
 * \brief Contains std::strings for each of DRCHUBO's joints in the URDF model, indexed according to <hubo.h>
 * NB: The size of the array must be equal to HUBO_JOINT_COUNT as defined in <hubo.h>
 */
const std::string DRCHUBO_URDF_JOINT_NAMES[] =
{
	"TSY","NKY","NKP","", //HNR, HNP ?
	"LSP","LSR","LSY","LEP","LWY","LWR","LWP",
	"RSP","RSR","RSY","REP","RWY","RWR","RWP","",
	"LHY","LHR","LHP","LKP","LAP","LAR","",
	"RHY","RHR","RHP","RKP","RAP","RAR",
	"RF1","RF2","RF3","","RWR_dummy",
	"LF1","LF2","LF3","","LWR_dummy",
};

/**
 * \var DRCHUBO_JOINT_NAME_TO_INDEX
 * \brief Maps std::strings for each of Hubo's joints to their indices according to <hubo.h>
 * NB: Requires C++11 to compile
 */
const std::map<std::string, int> DRCHUBO_JOINT_NAME_TO_INDEX = {
	{"TSY",WST},{"NKY",NKY},{"NKP",NK1},{"NK2",NK2},
	{"LSP",LSP},{"LSR",LSR},{"LSY",LSY},{"LEB",LEB},{"LWY",LWY},{"LWR",LWR},{"LWP",LWP},
	{"RSP",RSP},{"RSR",RSR},{"RSY",RSY},{"REB",REB},{"RWY",RWY},{"RWR",RWR},{"RWP",RWP},
	{"LHY",LHY},{"LHR",LHR},{"LHP",LHP},{"LKN",LKN},{"LAP",LAP},{"LAR",LAR},
	{"RHY",RHY},{"RHR",RHR},{"RHP",RHP},{"RKN",RKN},{"RAP",RAP},{"RAR",RAR},
	{"RF1",RF1},{"RF2",RF2},{"RF3",RF3},{"RF4",RF4},{"RF5",RF5},
	{"LF1",LF1},{"LF2",LF2},{"LF3",LF3},{"LF4",LF4},{"LF5",LF5}
};

/**
 * \var DRCHUBO_JOINT_NAME_TO_LIMB_POSITION
 * \brief Maps std::strings for each of Hubo's joints to their place in the limb's chain according to <hubo.h>
 * NB: Requires C++11 to compile
 */
const std::map<std::string, unsigned> DRCHUBO_JOINT_NAME_TO_LIMB_POSITION = {
	{"LSP",0},{"LSR",1},{"LSY",2},{"LEB",3},{"LWY",4},{"LWR",5},{"LWP",6},
	{"RSP",0},{"RSR",1},{"RSY",2},{"REB",3},{"RWY",4},{"RWR",5},{"RWP",6},
	{"LHY",0},{"LHR",1},{"LHP",2},{"LKN",3},{"LAP",4},{"LAR",5},
	{"RHY",0},{"RHR",1},{"RHP",2},{"RKN",3},{"RAP",4},{"RAR",5}
};

/**
 * \var DRCHUBO_JOINT_INDEX_TO_LIMB_POSITION
 * \brief Maps indices for each of Hubo's joints to their place in the limb's chain according to <hubo.h>
 * NB: Requires C++11 to compile
 */
const unsigned DRCHUBO_JOINT_INDEX_TO_LIMB_POSITION[] = {
	0,0,0,0,
	0,1,2,3,4,5,6,
	0,1,2,3,4,5,6,0,
	0,1,2,3,4,5,0,
	0,1,2,3,4,5,
	0,0,0,0,0,
	0,0,0,0,0
};

/**
 * \var DRCHUBO_URDF_FINGER_NAMES
 * \brief Lists the finger names for Hubo's hand for hubo_urdf
 */
const std::string DRCHUBO_URDF_FINGER_NAMES[] =
{
	"Thumb","Index","Pinky"
};

/**
 * \var DRCHUBO_URDF_FINGER_LINK_NAMES
 * \brief Lists the finger link names for Hubo's hand for hubo_urdf
 */
const std::string DRCHUBO_URDF_FINGER_LINK_NAMES[] =
{
	"Proximal","Medial","Distal"
};
#endif //DRCHUBO_JOINT_NAMES_H
