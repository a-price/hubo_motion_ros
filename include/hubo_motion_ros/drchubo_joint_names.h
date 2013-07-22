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
#include <vector>

#include <hubo.h>


/**
 * \var DRCHUBO_JOINT_NAMES
 * \brief Contains std::strings for each of Hubo's joints, indexed according to <hubo.h>
 * NB: The size of the array must be equal to HUBO_JOINT_COUNT as defined in <hubo.h>
 */
const std::string DRCHUBO_JOINT_NAMES[] =
{
	"TSY","NKY","NK1","NK2",
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
	"TSY","NKY","NK1","NK2",
	"LSP","LSR","LSY","LEP","LWY","LWR","LWP",
	"RSP","RSR","RSY","REP","RWY","RWR","RWP","",
	"LHY","LHR","LHP","LKP","LAP","LAR","",
	"RHY","RHR","RHP","RKP","RAP","RAR",
	"RF1","RF2","RF3","","",
	"LF1","LF2","LF3","","",
};

/**
 * \var DRCHUBO_JOINT_NAME_TO_INDEX
 * \brief Maps std::strings for each of Hubo's joints to their indices according to <hubo.h>
 * NB: Requires C++11 to compile
 */
const std::map<std::string, int> DRCHUBO_JOINT_NAME_TO_INDEX = {
	{"TSY",WST},{"NKY",NKY},{"NK1",NK1},{"NK2",NK2},
	{"LSP",LSP},{"LSR",LSR},{"LSY",LSY},{"LEP",LEB},{"LWY",LWY},{"LWR",LWR},{"LWP",LWP},
	{"RSP",RSP},{"RSR",RSR},{"RSY",RSY},{"REP",REB},{"RWY",RWY},{"RWR",RWR},{"RWP",RWP},
	{"LHY",LHY},{"LHR",LHR},{"LHP",LHP},{"LKP",LKN},{"LAP",LAP},{"LAR",LAR},
	{"RHY",RHY},{"RHR",RHR},{"RHP",RHP},{"RKP",RKN},{"RAP",RAP},{"RAR",RAR},
	{"RF1",RF1},{"RF2",RF2},{"RF3",RF3},{"RF4",RF4},{"RF5",RF5},
	{"LF1",LF1},{"LF2",LF2},{"LF3",LF3},{"LF4",LF4},{"LF5",LF5}
};

/**
 * \var DRCHUBO_JOINT_NAME_TO_LIMB_POSITION
 * \brief Maps std::strings for each of Hubo's joints to their place in the limb's chain according to <hubo.h>
 * NB: Requires C++11 to compile
 */
const std::map<std::string, unsigned> DRCHUBO_JOINT_NAME_TO_LIMB_POSITION = {
	{"LSP",0},{"LSR",1},{"LSY",2},{"LEP",3},{"LWY",4},{"LWP",5},{"LWR",6},
	{"RSP",0},{"RSR",1},{"RSY",2},{"REP",3},{"RWY",4},{"RWP",5},{"RWR",6},
	{"LHY",0},{"LHR",1},{"LHP",2},{"LKP",3},{"LAP",4},{"LAR",5},
	{"RHY",0},{"RHR",1},{"RHP",2},{"RKP",3},{"RAP",4},{"RAR",5}
};

/**
 * \var DRCHUBO_JOINT_NAME_TO_LIMB_
 * \brief Maps std::strings for each of Hubo's joints to their limb according to <hubo.h>
 * NB: Requires C++11 to compile
 */
const std::map<std::string, unsigned> DRCHUBO_JOINT_NAME_TO_LIMB = {
	{"LSP",LEFT},{"LSR",LEFT},{"LSY",LEFT},{"LEP",LEFT},{"LWY",LEFT},{"LWR",LEFT},{"LWP",LEFT},
	{"RSP",RIGHT},{"RSR",RIGHT},{"RSY",RIGHT},{"REP",RIGHT},{"RWY",RIGHT},{"RWR",RIGHT},{"RWP",RIGHT},
	{"LHY",LEFT+2},{"LHR",LEFT+2},{"LHP",LEFT+2},{"LKP",LEFT+2},{"LAP",LEFT+2},{"LAR",LEFT+2},
	{"RHY",RIGHT+2},{"RHR",RIGHT+2},{"RHP",RIGHT+2},{"RKP",RIGHT+2},{"RAP",RIGHT+2},{"RAR",RIGHT+2}
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
	"1","2","3"
};

/**
 * \var DRCHUBO_URDF_FINGER_LINK_NAMES
 * \brief Lists the finger link names for Hubo's hand for hubo_urdf
 */
const std::string DRCHUBO_URDF_FINGER_LINK_NAMES[] =
{
	"1","2","3"
};

/**
 * \var DRCHUBO_ARM_INDEX_TO_NAMES
 * \brief Lists the joint names for each arm
 */
const std::map<unsigned, std::vector<std::string> > DRCHUBO_ARM_INDEX_TO_NAMES = {
	{LEFT, {"LSP","LSR","LSY","LEP","LWY","LWR","LWP"}},
	{RIGHT, {"RSP","RSR","RSY","REP","RWY","RWR","RWP"}}
};
#endif //DRCHUBO_JOINT_NAMES_H
