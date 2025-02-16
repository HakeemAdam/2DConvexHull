#ifndef __CONVEXHULL_H__
#define __CONVEXHULL_H__

#include <SOP/SOP_Node.h>

class ConvexHull : public SOP_Node
{
public:
    static OP_Node *myConstructor(OP_Network *net, const char *name, OP_Operator* op);
    static const PRM_Template convexHullParms;

protected:
    ConvexHull(OP_Network *net, const char *name, OP_Operator* op);
    virtual ~ConvexHull();
    virtual OP_ERROR cookMySop(OP_Context &context);

private:
    int angleOrientation(UT_Vector3 &v1, UT_Vector3 &v2, UT_Vector3 &v3);
    
    
};


#endif