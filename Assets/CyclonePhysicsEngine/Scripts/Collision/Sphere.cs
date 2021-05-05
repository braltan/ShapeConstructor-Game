using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Sphere : MonoBehaviour
{
    public Cyclone.CollisionSphere collisionSphere = new Cyclone.CollisionSphere();
    // Start is called before the first frame update
    void Start()
    {
        collisionSphere.body = new Cyclone.RigidBody();
        collisionSphere.body.Mass=1.5f;
        collisionSphere.body.SetVelocity(0.0f, 0.0f, 0.0f);
        collisionSphere.body.SetAcceleration(0.0f, -10.0f, 0.0f);
        collisionSphere.body.SetDamping(0.99f, 0.8f);
        collisionSphere.radius = 0.2f;

        collisionSphere.body.SetPosition(0.0f, 5.0f, 0.0f);

        collisionSphere.body.SetCanSleep(false);
        collisionSphere.body.SetAwake();
        collisionSphere.body.CalculateDerivedData();
        collisionSphere.calculateInternals();

    }

    // Update is called once per frame
    void Update()
    {
        collisionSphere.body.Integrate(Time.deltaTime);
        collisionSphere.calculateInternals();
        collisionSphere.body.AddForce(new Cyclone.Math.Vector3(1, 0, 0));
        PositionGameObject();
    }
    private void PositionGameObject()
    {
        UnityEngine.Vector3 pos = new UnityEngine.Vector3((float)collisionSphere.body.Position.x, (float)collisionSphere.body.Position.y, (float)collisionSphere.body.Position.z);
        UnityEngine.Quaternion rot = new UnityEngine.Quaternion((float)collisionSphere.body.Orientation.i, (float)collisionSphere.body.Orientation.j, (float)collisionSphere.body.Orientation.k, (float)collisionSphere.body.Orientation.r);
        if (!float.IsNaN(pos.x) && !float.IsNaN(pos.y) && !float.IsNaN(pos.z))
        {
            transform.position = pos;
        }
        if (!float.IsNaN(rot.x) && !float.IsNaN(rot.y) && !float.IsNaN(rot.z))
        {
            transform.rotation = rot;
        }
        Debug.Log(collisionSphere.body.Position.y);
        
    }
}
