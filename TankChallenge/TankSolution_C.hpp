#ifndef TC_SOLUTION_hXX
#define TC_SOLUTION_HXX

#include "API.h"
#include "math.h"
#include "stdio.h"


/* - ---------------------------------------------------------------------- - */
// Simple AI for Honeypot Code Challenge
//
// Very verbose code, C compiled as C++. 
// Mixed style, uses Cpp features as I see fit.
// Many bad choises and style, you should probably not learn from this : ]
/* - ---------------------------------------------------------------------- - */


/* Forwards */
class API;

typedef struct TC_TrnOpts_t {
	int VerboseRun_b;
} TC_TrnOpts_t;

#define TC_MEM_REALLOC realloc
#define TC_MEM_CALLOC calloc
#define TC_MEM_ALLOC malloc
#define TC_MEM_FREE free

/* Debug print routine */
#define LOG_SEPARATOR( ) printf( "%s\n", "--------------------")
#define LOG_PRINT_FROM( f, msg ) printf( "[ %s ] %s\n", f, msg )
#define LOG_VALUE( f, val ) printf( "[ %s ] \t- %d \n", f, val )
#define LOG_PRINT( msg ) printf( "%s\n", msg )

/* - ---------------------------------------------------------------------- - */
// Double Linked list
/* - ---------------------------------------------------------------------- - */

typedef struct Node_t {
	Node_t * lstnext;
	Node_t * lstprev;
	void * nodeData;
} Node_t;

typedef struct TC_List_t {
	int objrefs;
	Node_t * firstNode;
	Node_t * lastNode;
} TC_List_t;

int CLst_GetObjCount( TC_List_t * list ) { 
	return ( list->objrefs ); 
}

void * CLst_Peek( TC_List_t * list ) {
	return ( list->firstNode->nodeData );
}

void * CLst_Pop( TC_List_t * list ) {
	return ( list->lastNode->nodeData );
}

void CLst_Append( TC_List_t * list, void * lstdata, int default_calloc ) {
	Node_t * lstnode = ( Node_t* ) TC_MEM_CALLOC( default_calloc, sizeof( Node_t ) );	
	if ( lstnode != nullptr && list != nullptr ) {
		if ( list->firstNode == nullptr && list->lastNode == nullptr ) {
			list->firstNode = lstnode; list->lastNode = lstnode;
		}

		lstnode->nodeData = lstdata;
		++list->objrefs;

		if ( list->objrefs > 1 ) {
			list->lastNode->lstprev = list->lastNode;
			list->lastNode->lstnext = lstnode;
			list->lastNode = lstnode;
		}
	}
}

void CLst_Find( TC_List_t * list, void * nodeData, Node_t **outnode ) {  
	Node_t * lstnode = list->firstNode;
	while ( lstnode != nullptr ) {	
		if ( lstnode->nodeData == nodeData ) {   
			(*outnode) = *&lstnode;		  
			break;
		}
		lstnode = lstnode->lstnext;
	}
}

/* All instructions possible for the tank to perform */
typedef enum TC_Instruction {
	T_FORWARDS = 0, // Move tank one node forward
	T_BACKWARDS, 	// Face opposite direction ( Turn left x2 ). 
	T_RIGHT, 		// Turn tank to face right
	T_LEFT, 		// Turn tank to face left

	T_PASSIVE_TURN, // This is a passive turn. Dont do anything.
	T_FIRE_CANNON,  // Atempt to destory a target this turn.
	T_DETECT_TRG, 	// Detect if obstacle ahead is a valid target.
	T_SCAN_ENV 		// Do a full lidar sweep.
} TC_Instruction;

const static int TC_MAX_INSTR_AMOUNT 	= 8; 	// Max amount of available instructions.
const static int TC_MAX_INSTR_QUEUE 	= 2; 	// Max instruction queue size.
const static int TC_MAX_NEIGHBOURS 		= 4; 	// Max number of neighbouring Nodes ( tank perspective ).

const static int TC_MAX_LEVEL_WIDTH 	= 20; 	// Estimated max map size ( w ).
const static int TC_MAX_LEVEL_HEIGHT 	= 20;   // Estimated max map size ( h ).

const int MAGIC_UNDEFINED_VALUE 		= 999; 	// No value placeholder.

typedef struct TC_Instruction_t {
	const char * SimpleInstrName;

	int InstrDone;
	int VitalInstructions;
	int Instructions[ TC_MAX_INSTR_QUEUE ];
} TC_Instruction_t;

typedef struct TC_MapNode_t {
	int Visited_b;
	int Obstacle_b;

	int NodeDist;
	int NodeInstr;
} TC_MapNode_t;

typedef struct TC_TankEntity_t {
	int FuelAmount;

	int CurrentPerformingInstrStep;
	int CurrentPerformingInstr;
	int CurrentForbiddenInstr;

	TC_Instruction_t Instruction;
	TC_MapNode_t NeighbouringNode[ TC_MAX_NEIGHBOURS ];
} TC_TankEntity_t;

const TC_TrnOpts_t TC_Trn_Opts_t = { 1 };
const TC_Instruction_t TC_Instructions[ TC_MAX_INSTR_AMOUNT ] = {
	{ "Turn_Forward", 0, 1, { T_FORWARDS, 0 }},
	{ "Turn_Backward", 0, 2, { T_LEFT, T_LEFT }},
	{ "Turn_Right", 0, 1, { T_RIGHT, 0 }},
	{ "Turn_Left", 0, 1, { T_LEFT, 0 }},

	{ "Turn_Passive", 0, 1, { T_PASSIVE_TURN, 0 }},
	{ "Turn_Destory", 0, 1, { T_FIRE_CANNON, 0 }},
	{ "Turn_Detect", 0, 1, { T_DETECT_TRG, 0 }},
	{ "Turn_Scan", 0, 1, { T_SCAN_ENV, 0 }}
};

int initedTankEntity_b = 0;
TC_TankEntity_t * TankEntity;
TC_List_t * NodeList;

int PreTurnInit( const TC_TrnOpts_t * opts ) {
	int errorCode = 1;

	TankEntity = ( TC_TankEntity_t* ) TC_MEM_ALLOC( sizeof( TC_TankEntity_t ) );
	TC_Instruction_t instruction = TC_Instructions[ T_PASSIVE_TURN ];
	if( TankEntity == 0 || instruction.SimpleInstrName == 0 ) {
		errorCode == 0;
		if ( opts->VerboseRun_b ) {
			LOG_PRINT_FROM( "PreTurnInit" , "Failed to allocate memory for tankEntity or Instructions." );
		}
	}

	TankEntity->CurrentPerformingInstrStep = 0;
	TankEntity->FuelAmount = API::currentFuel();
	TankEntity->CurrentPerformingInstr = T_PASSIVE_TURN;
	TankEntity->CurrentForbiddenInstr = MAGIC_UNDEFINED_VALUE;
	TankEntity->Instruction = TC_Instructions[ T_PASSIVE_TURN ];
	return errorCode;
}

void TurnLogicRoutine( const TC_TrnOpts_t * opts ) {
	if ( !initedTankEntity_b ) {
		NodeList = ( TC_List_t* ) TC_MEM_CALLOC( 0, sizeof( TC_List_t ));
		initedTankEntity_b = PreTurnInit( opts );
	}

	int closestDistSq = 999;
	int bestDirection = 0;
	int hasTarget = 0;

	/* Select the best instruction */
	if ( TankEntity->Instruction.InstrDone || TankEntity->Instruction.Instructions[ 0 ] == T_PASSIVE_TURN ) {
		TankEntity->NeighbouringNode[ T_FORWARDS ].NodeDist = API::lidarFront( );	
		TankEntity->NeighbouringNode[ T_BACKWARDS ].NodeDist = API::lidarBack( );	
		TankEntity->NeighbouringNode[ T_RIGHT ].NodeDist = API::lidarRight( );	
		TankEntity->NeighbouringNode[ T_LEFT ].NodeDist = API::lidarLeft( );

		for ( int iter = TC_MAX_NEIGHBOURS - 1; iter >= 0; --iter ) {
			hasTarget = (( int ) API::identifyTarget( ));
			int collidedBlock = ( TankEntity->NeighbouringNode[ bestDirection ].NodeDist == 1 );
			if ( collidedBlock && !hasTarget ) {
				TankEntity->CurrentForbiddenInstr = bestDirection;
			}

			int distance = TankEntity->NeighbouringNode[ iter ].NodeDist;
			int distanceSq = ( distance * distance );

			int forbiddenDirection = TankEntity->CurrentForbiddenInstr;
			if ( closestDistSq > distanceSq && iter != forbiddenDirection ) {
				bestDirection = iter;
				closestDistSq = distanceSq;
			}
		}

		/* Found new instruction */
		TC_Instruction_t hasTargetInstruction = TC_Instructions[ T_FIRE_CANNON ];
		TankEntity->Instruction = ( !hasTarget ) ? TC_Instructions[ bestDirection ] : hasTargetInstruction;
	}

	int passiveTurn = ( TankEntity->Instruction.Instructions[ 0 ] == T_PASSIVE_TURN );
	int turnMaxInstructionSteps= TankEntity->Instruction.VitalInstructions;
	int currentInstructionStep = TankEntity->CurrentPerformingInstrStep; 

	/* Perform instruction - move tank */
	if ( !TankEntity->Instruction.InstrDone && initedTankEntity_b ) { 
		int reachedMaxInstructions = ( currentInstructionStep >= turnMaxInstructionSteps );	
		if ( !passiveTurn && !reachedMaxInstructions ) {	
			int instruction = TankEntity->Instruction.Instructions[ currentInstructionStep ];

			switch ( instruction ) {
				case T_FORWARDS: 	API::moveForward( ); break;
				case T_BACKWARDS: 	API::moveBackward( ); break;
				case T_LEFT: 		API::turnLeft( ); break;
				case T_RIGHT: 		API::turnRight( ); break;
				case T_FIRE_CANNON: API::fireCannon( ); break;
				case T_DETECT_TRG: 	API::identifyTarget( ); break;

				case T_SCAN_ENV:	/* Full lidar sweep */ break;
				case T_PASSIVE_TURN:/* Do nothing this turn*/ break;

				default: LOG_PRINT( "Failed to find Instruction to perform or passive turn" ); 
			}
		}

		++TankEntity->CurrentPerformingInstrStep;
	}	

	int lastInstruction = TankEntity->Instruction.Instructions[ TankEntity->CurrentPerformingInstrStep - 1 ];
	int nextInstructionStep = TankEntity->CurrentPerformingInstrStep;
	int finishedTurnMove = ( nextInstructionStep == turnMaxInstructionSteps || passiveTurn );

	TankEntity->Instruction.InstrDone = finishedTurnMove; 
	TankEntity->CurrentPerformingInstrStep = ( finishedTurnMove ) ? 0 : TankEntity->CurrentPerformingInstrStep;;
}

void TurnStart( ) { LOG_SEPARATOR(); LOG_PRINT( " - Turn starting -" ); }
void TurnEnd( ) { LOG_PRINT( " - Turn ending -" ); LOG_SEPARATOR(); }

class Solution {
	public:
		Solution() { /* Empty by design */ }

		void update( ) {
			TurnStart( );

			TurnLogicRoutine( &TC_Trn_Opts_t );
			if ( TC_Trn_Opts_t.VerboseRun_b ) {
				LOG_VALUE( "Turn Instruction ", TankEntity->Instruction.Instructions[ TankEntity->CurrentPerformingInstrStep ] );
				LOG_VALUE( "Turn InstructionStep", TankEntity->CurrentPerformingInstrStep );
				LOG_VALUE( "Turn forbidden", TankEntity->CurrentForbiddenInstr );
				LOG_VALUE( "Instruction Done", TankEntity->Instruction.InstrDone);
				LOG_PRINT( TankEntity->Instruction.SimpleInstrName );
			}

			TurnEnd( );
		}
};

#endif
