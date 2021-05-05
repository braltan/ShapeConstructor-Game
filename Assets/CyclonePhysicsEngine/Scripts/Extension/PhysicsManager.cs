using Cyclone;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsManager : MonoBehaviour
{
    public GameObject[] boxData = new GameObject[5];
    public enum CollisionType {boxAndBox,boxAndHalfSpace,boxAndSphere};
    public CollisionType collisionType;
    public static int maxContacts = 256;
    public CollisionData cData;
    ContactResolver resolver;


    void Start()
    {
        resolver = new ContactResolver(maxContacts * 8);
        cData = new CollisionData();
    }

    void FixedUpdate()
    {
        generateContacts();
        if (cData.contactCount > 0)
        resolver.resolveContacts(cData.contacts, cData.contactCount, Time.deltaTime);
    }
    public void generateContacts()
    {
        CollisionPlane plane = new CollisionPlane();
        plane.direction = new Cyclone.Math.Vector3(0.0, 1.0, 0.0);
        plane.offset = 0;

        cData.reset(maxContacts);
        cData.friction = (double)0.9;
        cData.restitution = (double)0.1;
        cData.tolerance = (double)0.1;

        if(collisionType == CollisionType.boxAndBox)
           CollisionDetector.boxAndBox((CollisionBox)(boxData[0].GetComponent<Rigidbody>().physicsObject),(CollisionBox)(boxData[1].GetComponent<Rigidbody>().physicsObject), cData);
        else if(collisionType == CollisionType.boxAndHalfSpace)
           CollisionDetector.boxAndHalfSpace((CollisionBox)(boxData[4].GetComponent<Rigidbody>().physicsObject), plane, cData);
        else if (collisionType == CollisionType.boxAndSphere)
          CollisionDetector.boxAndSphere((CollisionBox)(boxData[3].GetComponent<Rigidbody>().physicsObject), (CollisionSphere)(boxData[2].GetComponent<Rigidbody>().physicsObject), cData);
    }
}
