using Cyclone.Math;
using System.Collections.Generic;

namespace Cyclone
{
    public class CollisionPrimitive
    {
        public RigidBody body = new RigidBody();
        public Matrix4 offset = new Matrix4();
        public Matrix4 transform = new Matrix4();
        public Vector3 getAxis(int index)
        {
            return transform.GetAxisVector(index);
        }

        public Matrix4 getTransform()
        {
            return transform;
        }
        public void calculateInternals()
        {
            this.transform = body.GetTransform() * offset;
        }

    }
    public class CollisionSphere : CollisionPrimitive
        {
        public double radius;
        }
      public class CollisionPlane
    {
        public Vector3 direction;
        public double offset;
    }
    public class CollisionBox : CollisionPrimitive
    {
        public Vector3 halfSize;
    }

    public class IntersectionTests
    {
        public static bool sphereAndHalfSpace(CollisionSphere sphere, CollisionPlane plane)
        {
            double ballDistance =
                plane.direction *
                sphere.getAxis(3) -
                sphere.radius;

            return ballDistance <= plane.offset;
        }
        public static bool sphereAndSphere(CollisionSphere one, CollisionSphere two)
        {
            Vector3 midline = one.getAxis(3) - two.getAxis(3);

            return midline.SquareMagnitude <
                (one.radius + two.radius) * (one.radius + two.radius);
        }
        static double transformToAxis(CollisionBox box,Vector3 axis)
        { 
    return
        box.halfSize.x* System.Math.Abs(axis* box.getAxis(0)) +
        box.halfSize.y * System.Math.Abs(axis* box.getAxis(1)) +
        box.halfSize.z * System.Math.Abs(axis* box.getAxis(2));
           }
       static  bool overlapOnAxis(CollisionBox one,CollisionBox two,Vector3 axis,Vector3 toCentre)
{
            double oneProject = transformToAxis(one, axis);
            double twoProject = transformToAxis(two, axis);

            double distance = System.Math.Abs(toCentre * axis);

    return (distance<oneProject + twoProject);
}
     
        public static bool boxAndHalfSpace(CollisionBox box, CollisionPlane plane)
        {
            double projectedRadius = transformToAxis(box, plane.direction);

            double boxDistance =
                plane.direction *
                box.getAxis(3) -
                projectedRadius;

            return boxDistance <= plane.offset;
        }
    }
    public class CollisionData
    {
        public Contact contactArray;
        public Contact [] contacts;

        public int contactsLeft;

        public int contactCount;

        public double friction;

        public double restitution;

        public double tolerance;
       public bool hasMoreContacts()
        {
            return contactsLeft > 0;
        }

        public void reset(int maxContacts)
        {
            contactsLeft = maxContacts;
            contactCount = 0;
            contacts = new Contact[maxContacts];
            for (int i = 0; i < maxContacts; i++)
            {
                contacts[i] = new Contact();
            }
        }

        public void addContacts(int count)
        {
            contactsLeft -= count;
            contactCount += count;

        }
    }
    public class CollisionDetector
    {
        public static int sphereAndTruePlane(CollisionSphere sphere,CollisionPlane plane,CollisionData  data)
        {
            if (data.contactsLeft <= 0) return 0;

            Vector3 position = sphere.getAxis(3);

            double centreDistance = plane.direction * position - plane.offset;

            if (centreDistance * centreDistance > sphere.radius * sphere.radius)
            {
                return 0;
            }

            Vector3 normal = plane.direction;
            double penetration = -centreDistance;
            if (centreDistance < 0)
            {
                normal *= -1;
                penetration = -penetration;
            }
            penetration += sphere.radius;

            Contact contact = data.contacts[data.contactCount];
            contact.contactNormal = normal;
            contact.penetration = penetration;
            contact.contactPoint = position - plane.direction * centreDistance;
            contact.setBodyData(sphere.body, null,
                data.friction, data.restitution);

            data.addContacts(1);
            return 1;
        }
        public static int sphereAndHalfSpace(CollisionSphere sphere, CollisionPlane plane, CollisionData data)
        {
            if (data.contactsLeft <= 0) return 0;

            Vector3 position = sphere.getAxis(3);

            double ballDistance =
                plane.direction * position -
                sphere.radius - plane.offset;

            if (ballDistance >= 0) return 0;

            Contact contact = data.contacts[data.contactCount];
            contact.contactNormal = plane.direction;
            contact.penetration = -ballDistance;
            contact.contactPoint =
                position - plane.direction * (ballDistance + sphere.radius);
            contact.setBodyData(sphere.body, null,
                data.friction, data.restitution);

            data.addContacts(1);
            return 1;
        }
        public static int sphereAndSphere(CollisionSphere one, CollisionSphere two, CollisionData data)
        {
            if (data.contactsLeft <= 0) return 0;

            Vector3 positionOne = one.getAxis(3);
            Vector3 positionTwo = two.getAxis(3);

            Vector3 midline = positionOne - positionTwo;
            double size = midline.Magnitude;

            if (size <= 0.0f || size >= one.radius + two.radius)
            {
                return 0;
            }

            Vector3 normal = midline * (((double)1.0) / size);

            return 1;
        }
        public static int boxAndHalfSpace(CollisionBox box, CollisionPlane plane, CollisionData data)
        {
            if (data.contactsLeft <= 0) return 0;

            if (!IntersectionTests.boxAndHalfSpace(box, plane))
            {
                return 0;
            }

             double [,] mults = new double[8,3]{{1,1,1},{-1,1,1},{1,-1,1},{-1,-1,1},
                               {1,1,-1},{-1,1,-1},{1,-1,-1},{-1,-1,-1}};

        Contact [] contact = data.contacts;
        int contactsUsed = 0;
    for (int i = 0; i< 8; i++) {

        Vector3 vertexPos = new Vector3(mults[i,0], mults[i,1], mults[i,2]);
        vertexPos.ComponentProductUpdate(box.halfSize);
        vertexPos = box.transform.Transform(vertexPos);

        double vertexDistance = vertexPos * plane.direction;

        if (vertexDistance <= plane.offset)
        {
            contact[contactsUsed].contactPoint = plane.direction;
            contact[contactsUsed].contactPoint *= (vertexDistance-plane.offset);
            contact[contactsUsed].contactPoint += vertexPos;
            contact[contactsUsed].contactNormal = plane.direction;
            contact[contactsUsed].penetration = plane.offset - vertexDistance;

            contact[contactsUsed].setBodyData(box.body, null,
                data.friction, data.restitution);

            contactsUsed++;
            if (contactsUsed == (int) data.contactsLeft) return contactsUsed;
        }
}
        data.addContacts(contactsUsed);
        return contactsUsed;
        }
    
       public static int boxAndBox(CollisionBox one, CollisionBox two, CollisionData data)
        {
            Vector3 toCentre = two.getAxis(3) - one.getAxis(3);

            double pen = double.MaxValue;
            int best = 0xffffff;

            if (!tryAxis(one, two, one.getAxis(0), toCentre, 0,ref pen, ref best)) return 0;
            if (!tryAxis(one, two, one.getAxis(1), toCentre, 1, ref pen, ref best)) return 0;
            if (!tryAxis(one, two, one.getAxis(2), toCentre, 2, ref pen, ref best)) return 0;

            if (!tryAxis(one, two, one.getAxis(0), toCentre, 3, ref pen, ref best)) return 0;
            if (!tryAxis(one, two, one.getAxis(1), toCentre, 4, ref pen, ref best)) return 0;
            if (!tryAxis(one, two, one.getAxis(2), toCentre, 5, ref pen, ref best)) return 0;

            int bestSingleAxis = best;

            if (!tryAxis(one, two, one.getAxis(0).VectorProduct(two.getAxis(0)), toCentre, 6, ref pen, ref best)) return 0;
            if (!tryAxis(one, two, one.getAxis(0).VectorProduct(two.getAxis(1)), toCentre, 7, ref pen, ref best)) return 0;
            if (!tryAxis(one, two, one.getAxis(0).VectorProduct(two.getAxis(2)), toCentre, 8, ref pen, ref best)) return 0;
            if (!tryAxis(one, two, one.getAxis(1).VectorProduct(two.getAxis(0)), toCentre, 9, ref pen, ref best)) return 0;
            if (!tryAxis(one, two, one.getAxis(1).VectorProduct(two.getAxis(1)), toCentre, 10, ref pen, ref best)) return 0;
            if (!tryAxis(one, two, one.getAxis(1).VectorProduct(two.getAxis(2)), toCentre,11, ref pen, ref best)) return 0;
            if (!tryAxis(one, two, one.getAxis(2).VectorProduct(two.getAxis(0)), toCentre, 12, ref pen, ref best)) return 0;
            if (!tryAxis(one, two, one.getAxis(2).VectorProduct(two.getAxis(1)), toCentre, 13, ref pen, ref best)) return 0;
            if (!tryAxis(one, two, one.getAxis(2).VectorProduct(two.getAxis(2)), toCentre, 14, ref pen, ref best)) return 0;

            if (Core.Equals(best, 0xffffff))
            {
                return int.MaxValue;
            }


            if (best < 3)
            {
                fillPointFaceBoxBox(one, two, toCentre, data, best, pen);
                data.addContacts(1);
                return 1;
            }
            else if (best < 6)
            {
                fillPointFaceBoxBox(two, one, toCentre * -1.0f, data, best - 3, pen);
                data.addContacts(1);
                return 1;
            }
            else
            {
                best -= 6;
                int oneAxisIndex = best / 3;
                int twoAxisIndex = best % 3;
                Vector3 oneAxis = new Vector3();
                oneAxis = one.getAxis(oneAxisIndex);
                Vector3 twoAxis = new Vector3();
                twoAxis = two.getAxis(twoAxisIndex);
                Vector3 axis = new Vector3();
                axis = oneAxis.VectorProduct(twoAxis);
                axis.Normalize();

                if (axis * toCentre > 0) axis = axis * -1.0f;

                Vector3 ptOnOneEdge = new Vector3();
                ptOnOneEdge = one.halfSize;
                Vector3 ptOnTwoEdge = new Vector3();
                ptOnTwoEdge = two.halfSize;

                for (int i = 0; i < 3; i++)
                {
                    if (i == 0)
                    {
                        if (i == oneAxisIndex) ptOnOneEdge.x =0 ;
                        else if (one.getAxis(i) * axis > 0) ptOnOneEdge.x =-ptOnOneEdge.x;

                        if (i == twoAxisIndex) ptOnTwoEdge.x = 0;
                        else if (two.getAxis(i) * axis < 0) ptOnTwoEdge.x = -ptOnTwoEdge.x;
                    }
                    else if(i==1)
                    {
                        if (i == oneAxisIndex) ptOnOneEdge.y = 0;
                        else if (one.getAxis(i) * axis > 0) ptOnOneEdge.y = -ptOnOneEdge.y;

                        if (i == twoAxisIndex) ptOnTwoEdge.y = 0;
                        else if (two.getAxis(i) * axis < 0) ptOnTwoEdge.y = -ptOnTwoEdge.y;
                    }
                    else
                    {
                        if (i == oneAxisIndex) ptOnOneEdge.z = 0;
                        else if (one.getAxis(i) * axis > 0) ptOnOneEdge.z = -ptOnOneEdge.z;

                        if (i == twoAxisIndex) ptOnTwoEdge.z = 0;
                        else if (two.getAxis(i) * axis < 0) ptOnTwoEdge.z = -ptOnTwoEdge.z;
                    }
                }

                ptOnOneEdge = one.transform * ptOnOneEdge;
                ptOnTwoEdge = two.transform * ptOnTwoEdge;

                Vector3 vertex = new Vector3();
                  vertex = contactPoint(
                    ptOnOneEdge, oneAxis, one.halfSize[oneAxisIndex],
                    ptOnTwoEdge, twoAxis, two.halfSize[twoAxisIndex],
                    bestSingleAxis > 2
                    );

                Contact [] contact = data.contacts;

                contact[data.contactCount].penetration = pen;
                contact[data.contactCount].contactNormal = axis;
                contact[data.contactCount].contactPoint = vertex;
                contact[data.contactCount].setBodyData(one.body, two.body,
                data.friction, data.restitution);
                data.addContacts(1);
                return 1;
            }
            return 0;
        }
   
        public static bool tryAxis(CollisionBox one,CollisionBox two,Vector3 axis,Vector3 toCentre,int index,ref double smallestPenetration ,ref int smallestCase)
        {
            if (axis.SquareMagnitude < 0.0001) return true;
            axis.Normalize();

            double penetration = penetrationOnAxis(one, two, axis, toCentre);

            if (penetration < 0) return false;
            if (penetration !=0 && penetration < smallestPenetration)
            {
                smallestPenetration = penetration;
                smallestCase = index;
            }
            return true;
        }
        public static double penetrationOnAxis(CollisionBox one,CollisionBox two, Vector3 axis, Vector3 toCentre)
{
            double oneProject = transformToAxis(one, axis);
            double twoProject = transformToAxis(two, axis);

            double distance = System.Math.Abs(toCentre * axis);

    return oneProject + twoProject - distance;
}
        public static double transformToAxis(CollisionBox box,Vector3 axis)
{
    return
        box.halfSize.x* System.Math.Abs(axis* box.getAxis(0)) +
        box.halfSize.y* System.Math.Abs(axis* box.getAxis(1)) +
        box.halfSize.z* System.Math.Abs(axis* box.getAxis(2));
}
      public static void fillPointFaceBoxBox(CollisionBox one, CollisionBox two,Vector3 toCentre,CollisionData data, int best, double pen)
{
        Contact [] contact = data.contacts;

        Vector3 normal = one.getAxis(best);
    if (one.getAxis(best) * toCentre > 0)
    {
        normal = normal* -1.0f;
    }

    Vector3 vertex = two.halfSize;
    if (two.getAxis(0) * normal< 0) vertex.x = -vertex.x;
    if (two.getAxis(1) * normal< 0) vertex.y = -vertex.y;
    if (two.getAxis(2) * normal< 0) vertex.z = -vertex.z;

    contact[data.contactCount].contactNormal = normal;
    contact[data.contactCount].penetration = pen;
    contact[data.contactCount].contactPoint = two.getTransform() * vertex;
    contact[data.contactCount].setBodyData(one.body, two.body,
        data.friction, data.restitution);
}
     public static Vector3 contactPoint( Vector3 pOne, Vector3 dOne, double oneSize, Vector3 pTwo, Vector3 dTwo,double twoSize, bool useOne)
{
    Vector3 toSt, cOne, cTwo;
        double dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
            double denom, mua, mub;

        smOne = dOne.SquareMagnitude;
    smTwo = dTwo.SquareMagnitude;
    dpOneTwo = dTwo* dOne;

        toSt = pOne - pTwo;
    dpStaOne = dOne* toSt;
        dpStaTwo = dTwo* toSt;

        denom = smOne* smTwo - dpOneTwo* dpOneTwo;

    if (System.Math.Abs(denom) < 0.0001f) {
        return useOne? pOne:pTwo;
    }

    mua = (dpOneTwo* dpStaTwo - smTwo* dpStaOne) / denom;
    mub = (smOne* dpStaTwo - dpOneTwo* dpStaOne) / denom;

    if (mua > oneSize ||
        mua< -oneSize ||
        mub> twoSize ||
        mub< -twoSize)
    {
        return useOne? pOne:pTwo;
    }
    else
    {
        cOne = pOne + dOne* mua;
cTwo = pTwo + dTwo* mub;

        return cOne* 0.5 + cTwo* 0.5;
    }
}

         public static int boxAndSphere(CollisionBox box, CollisionSphere sphere, CollisionData data)
         {
            Vector3 centre = sphere.getAxis(3);
            Vector3 relCentre = box.transform.TransformInverse(centre);

            if (System.Math.Abs(relCentre.x) - sphere.radius > box.halfSize.x ||
                System.Math.Abs(relCentre.y) - sphere.radius > box.halfSize.y ||
                System.Math.Abs(relCentre.z) - sphere.radius > box.halfSize.z)
            {
                return 0;
            }

            Vector3 closestPt = new Vector3(0,0,0);
            double dist;

            dist = relCentre.x;
            if (dist > box.halfSize.x) dist = box.halfSize.x;
            if (dist < -box.halfSize.x) dist = -box.halfSize.x;
            closestPt.x = dist;

            dist = relCentre.y;
            if (dist > box.halfSize.y) dist = box.halfSize.y;
            if (dist < -box.halfSize.y) dist = -box.halfSize.y;
            closestPt.y = dist;

            dist = relCentre.z;
            if (dist > box.halfSize.z) dist = box.halfSize.z;
            if (dist < -box.halfSize.z) dist = -box.halfSize.z;
            closestPt.z = dist;

            dist = (closestPt - relCentre).SquareMagnitude;
            if (dist > sphere.radius * sphere.radius) return 0;

            Vector3 closestPtWorld = box.transform.Transform(closestPt);

            Contact [] contact = data.contacts;
            contact[data.contactCount].contactNormal = (closestPtWorld - centre);
            contact[data.contactCount].contactNormal.Normalize();
            contact[data.contactCount].contactPoint = closestPtWorld;
            contact[data.contactCount].penetration = sphere.radius - System.Math.Sqrt(dist);
            contact[data.contactCount].setBodyData(box.body, sphere.body,
                data.friction, data.restitution);

            data.addContacts(1);
            return 1;
        }
         
}
}
