using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Cyclone;

public class BoardManager : MonoBehaviour
{
     int height, width;
    public GameObject[] boxData = new GameObject[2];
    public GameObject box;
    public GameObject sphere;
    public static int maxContacts = 256;
    Contact[] contacts;
    public CollisionData cData;
    ContactResolver resolver;
    // Start is called before the first frame update
    void Start()
    {
        resolver = new ContactResolver(1);
        cData = new CollisionData();
    
        cData.contacts = contacts;

    }

    // Update is called once per frame
    void Update()
    {
        generateContacts();
        if(cData.contactCount > 0)
        resolver.resolveContacts(cData.contacts, cData.contactCount, Time.deltaTime);
    }
    public void generateContacts()
    {
        CollisionPlane plane = new CollisionPlane();
        plane.direction = new Cyclone.Math.Vector3(0, 1, 0);
        plane.offset = 0;
        cData.reset(maxContacts);
        cData.friction = (double)0.9;
        cData.restitution = (double)0.1;
        cData.tolerance = (double)0.1;

        // CollisionDetector.boxAndBox(boxData[0].GetComponent<Box>().collisionBox, boxData[1].GetComponent<Box>().collisionBox, cData);
          CollisionDetector.boxAndSphere(box.GetComponent<Box>().collisionBox, sphere.GetComponent<Sphere>().collisionSphere, cData);
        //CollisionDetector.sphereAndHalfSpace(sphere.GetComponent<Sphere>().collisionSphere, plane, cData);
        /* for (int i = 0 ; i < boxData.Length; i++)
           {
              if (!cData.hasMoreContacts()) return;
              for(int j=0; j<boxData.Length; j++)
              {
                  if(j!=i)
                  CollisionDetector.boxAndBox(boxData[i].GetComponent<Box>().collisionBox, boxData[j].GetComponent<Box>().collisionBox, cData);
              }

           }*/
    }
}
