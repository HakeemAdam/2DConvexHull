#ifndef M_PI
#define M_PI 3.14159265358979323846


#include <UT/UT_DSOVersion.h>
#include "2DConvexHull.h"


#include <OP/OP_OperatorTable.h>
#include <OP/OP_Operator.h>
#include <OP/OP_AutoLockInputs.h>

#include <GU/GU_Detail.h>

#include <PRM/PRM_Include.h>
#include  <PRM/PRM_Default.h>
#include <PRM/PRM_Template.h>

static  PRM_Template convexHullParms[]=
    {

        PRM_Template()
    };

void newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(new OP_Operator(
        "PG::ConvexHull",
        "PG::ConvexHull",
        ConvexHull::myConstructor,
        convexHullParms,
        1,
        1,
        nullptr));
}

OP_Node* ConvexHull::myConstructor(OP_Network* net, const char* name, OP_Operator* op)
{
    return new ConvexHull(net, name, op);
}

ConvexHull::ConvexHull(OP_Network* net, const char* name, OP_Operator* op)
    :SOP_Node(net, name, op)
{
    mySopFlags.setManagesDataIDs(true);
}

ConvexHull::~ConvexHull()
=default;

OP_ERROR ConvexHull::cookMySop(OP_Context& context)
{
    OP_AutoLockInputs inputs(this);
    if(inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    duplicateSource(0, context);

    GA_RWHandleV3 pos_h(gdp->getP());
    if(!pos_h.isValid())
    return error();
    
    GA_Range pts_range = gdp->getPointRange();
    GA_Size numPoints = pts_range.getMaxEntries();

 
    GA_RWHandleF handle = gdp->addFloatTuple(GA_ATTRIB_POINT, "angle",1);
    for(GA_Iterator it(pts_range); !it.atEnd(); ++it)
         {
             GA_Offset ptoff = *it;

              // find angle from selected point to all points and store as attr
             GA_Index idx = gdp->pointIndex(ptoff);
             GA_Index firstPt =gdp->pointIndex(0);
             if(idx != firstPt)
             {
           
                 UT_Vector3 n = pos_h.get(ptoff);
                 UT_Vector3 m = pos_h.get(firstPt);
     
                 float d = atan2(m.x()-n.x(), m.z()-n.z())  * 180.0f / M_PI;;
            
                 if(handle.isValid())
                 {
                     handle.set(ptoff, d);
                 }
             }
         }


    return error();
}

int ConvexHull::angleOrientation(UT_Vector3& v1, UT_Vector3& v2, UT_Vector3& v3)
{
    float val = (v2.x()-v1.x()) * (v3.z()-v2.z()) - (v2.z()- v1.z()) * (v3.x()-v2.x());
    if (val==0)
    {
        return 0;
    }
    return (val > 0) ? 1 : 2;
}

#endif
