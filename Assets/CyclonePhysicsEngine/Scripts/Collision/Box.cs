using Cyclone.Math;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Box : MonoBehaviour
{
    public double accel,x;
    public Cyclone.CollisionBox collisionBox = new Cyclone.CollisionBox();
    // Start is called before the first frame update
    void Start()
    {
        collisionBox.body = new Cyclone.RigidBody();
        setState(x,transform.position.y,0.0);

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        collisionBox.body.Integrate(Time.deltaTime);
        collisionBox.calculateInternals();
        PositionGameObject();

    }
    void setState(double x,double y,double z)
    {
        collisionBox.body.SetPosition(0, -1.0f, 0);
        collisionBox.body.SetOrientation(1, 0, 0, 0);
        collisionBox.body.SetVelocity(0, 0, 0);
        collisionBox.body.SetRotation(new Cyclone.Math.Vector3(0, 0, 0));
        collisionBox.halfSize = new Cyclone.Math.Vector3(1, 1, 1);

        double mass = collisionBox.halfSize.x * collisionBox.halfSize.y * collisionBox.halfSize.z;
        collisionBox.body.Mass = mass;

        Matrix3 tensor = new Matrix3();
        tensor.SetBlockInertiaTensor(collisionBox.halfSize, mass);
        collisionBox.body.SetInertiaTensor(tensor);

        collisionBox.body.LinearDamping = 0.95f;
        collisionBox.body.AngularDamping=0.8f;
        collisionBox.body.ClearAccumulators();
        collisionBox.body.SetAcceleration(0, 0, 0);

        collisionBox.body.SetCanSleep(false);
        collisionBox.body.SetAwake();

        collisionBox.body.CalculateDerivedData();
        collisionBox.calculateInternals();
    }
    private void PositionGameObject()
    {
        
            UnityEngine.Vector3 scale = new UnityEngine.Vector3((float)collisionBox.halfSize.x, (float)collisionBox.halfSize.y, (float)collisionBox.halfSize.z);
            UnityEngine.Vector3 pos = new UnityEngine.Vector3((float)collisionBox.body.Position.x, (float)collisionBox.body.Position.y, (float)collisionBox.body.Position.z);
        UnityEngine.Quaternion rot = new UnityEngine.Quaternion((float)collisionBox.body.Orientation.i, (float)collisionBox.body.Orientation.j, (float)collisionBox.body.Orientation.k, (float)collisionBox.body.Orientation.r);
        if (!float.IsNaN(pos.x) && !float.IsNaN(pos.y) && !float.IsNaN(pos.z))
        {
            transform.position = pos;
        }
        if (!float.IsNaN(rot.x) && !float.IsNaN(rot.y) && !float.IsNaN(rot.z))
        {
            transform.rotation = rot;
        }
        if (!float.IsNaN(scale.x) && !float.IsNaN(scale.y) && !float.IsNaN(scale.z))
        {
            transform.localScale = scale;
        }

             
        
    }

}
