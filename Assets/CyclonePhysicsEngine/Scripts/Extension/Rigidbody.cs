using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Cyclone;
using Cyclone.Math;

public class Rigidbody : MonoBehaviour
{
     enum BodyType {box, sphere, triangle};
    [SerializeField]
    BodyType bodyType;

    public object physicsObject;

    Cyclone.Math.Vector3 position, velocity, rotation, scale, acceleration;

    [SerializeField]
    double linearDamping, angularDamping,mass;

    [SerializeField]
    bool gravity;

    void Start()
    {
        if(bodyType == BodyType.box)
        {
            physicsObject = new CollisionBox();
            setUnityValues();
            setStateBox();     
        }
        else if (bodyType == BodyType.sphere)
        {
            physicsObject = new CollisionSphere();
            setUnityValues();
            setStateSphere();
        }
    }


    void FixedUpdate()
    {
        if (bodyType == BodyType.box)
        {
            ((CollisionBox)physicsObject).body.Integrate(Time.deltaTime);
            ((CollisionBox)physicsObject).calculateInternals();
            PositionGameObject();
        }
        else if (bodyType == BodyType.sphere)
        {
            ((CollisionSphere)physicsObject).body.Integrate(Time.deltaTime);
            ((CollisionSphere)physicsObject).calculateInternals();
            PositionGameObject();
        }
    }

    void setStateBox()
    {
        ((CollisionBox)physicsObject).body.SetPosition(position.x,position.y,position.z);
        ((CollisionBox)physicsObject).body.SetVelocity(0,0,0);
        ((CollisionBox)physicsObject).body.SetOrientation(1, 0, 0, 0);
        ((CollisionBox)physicsObject).halfSize = new Cyclone.Math.Vector3(1, 1,1);

         double mass = ((CollisionBox)physicsObject).halfSize.x * ((CollisionBox)physicsObject).halfSize.y * ((CollisionBox)physicsObject).halfSize.z *8.0f;
        ((CollisionBox)physicsObject).body.Mass = mass;

        Matrix3 tensor = new Matrix3();
        tensor.SetBlockInertiaTensor(((CollisionBox)physicsObject).halfSize, mass);
        ((CollisionBox)physicsObject).body.SetInertiaTensor(tensor);

        ((CollisionBox)physicsObject).body.LinearDamping = linearDamping;
        ((CollisionBox)physicsObject).body.AngularDamping = angularDamping;
        ((CollisionBox)physicsObject).body.ClearAccumulators();
        ((CollisionBox)physicsObject).body.SetAcceleration(acceleration.x,acceleration.y,acceleration.z);

        ((CollisionBox)physicsObject).body.SetCanSleep(false);
        ((CollisionBox)physicsObject).body.SetAwake();
        
        ((CollisionBox)physicsObject).body.CalculateDerivedData();
        ((CollisionBox)physicsObject).calculateInternals();
    }
    void setStateSphere()
    {
        ((CollisionSphere)physicsObject).body.SetPosition(position.x, position.y, position.z);
        ((CollisionSphere)physicsObject).body.SetRotation(new Cyclone.Math.Vector3(rotation.x, rotation.y, rotation.z));
        ((CollisionSphere)physicsObject).body.SetOrientation(1, 0, 0, 0);

        //  double mass = ((CollisionBox)physicsObject).halfSize.x * ((CollisionBox)physicsObject).halfSize.y ;
        ((CollisionSphere)physicsObject).body.Mass = mass;
        ((CollisionSphere)physicsObject).radius = 0.2;
        Matrix3 tensor = new Matrix3();
        double coeff = 0.4f * ((CollisionSphere)physicsObject).body.Mass * ((CollisionSphere)physicsObject).radius * ((CollisionSphere)physicsObject).radius;
        tensor.SetInertiaTensorCoeffs(coeff, coeff, coeff);
        ((CollisionSphere)physicsObject).body.SetInertiaTensor(tensor);

        ((CollisionSphere)physicsObject).body.LinearDamping = linearDamping;
        ((CollisionSphere)physicsObject).body.AngularDamping = angularDamping;
        ((CollisionSphere)physicsObject).body.ClearAccumulators();
        ((CollisionSphere)physicsObject).body.SetAcceleration(acceleration.x, acceleration.y, acceleration.z);
        
        ((CollisionSphere)physicsObject).body.SetCanSleep(false);
        ((CollisionSphere)physicsObject).body.SetAwake();

        ((CollisionSphere)physicsObject).body.CalculateDerivedData();
        ((CollisionSphere)physicsObject).calculateInternals();
    }

    void setUnityValues()
    {
        position = new Cyclone.Math.Vector3(transform.position.x, transform.position.y, transform.position.z);
        UnityEngine.Vector3 euler = UnityEngine.Quaternion.ToEulerAngles(transform.rotation);
        rotation = new Cyclone.Math.Vector3(euler.x,euler.y,euler.z);
        scale = new Cyclone.Math.Vector3(transform.localScale.x, transform.localScale.y, transform.localScale.z);
        velocity = new Cyclone.Math.Vector3(0, 0, 0);
        if(gravity)
        acceleration = new Cyclone.Math.Vector3(0.0, -10.0, 0.0);
        else
        {
            acceleration = new Cyclone.Math.Vector3(0.0, 0.0, 0.0);
        }
    }
    private void PositionGameObject()
    {
        if (bodyType == BodyType.box)
        {
            UnityEngine.Vector3 pos = new UnityEngine.Vector3((float)((CollisionBox)physicsObject).body.Position.x, (float)((CollisionBox)physicsObject).body.Position.y, (float)((CollisionBox)physicsObject).body.Position.z);
            if (!float.IsNaN(pos.x) && !float.IsNaN(pos.y) && !float.IsNaN(pos.z))
            {
                transform.position = pos;
            }
               transform.rotation = UnityEngine.Quaternion.Euler(new UnityEngine.Vector3((float)((CollisionBox)physicsObject).body.Rotation.x, (float)((CollisionBox)physicsObject).body.Rotation.y, (float)((CollisionBox)physicsObject).body.Rotation.z));

        }

        else if (bodyType == BodyType.sphere)
        {
            UnityEngine.Vector3 pos = new UnityEngine.Vector3((float)((CollisionSphere)physicsObject).body.Position.x, (float)((CollisionSphere)physicsObject).body.Position.y, (float)((CollisionSphere)physicsObject).body.Position.z);

            if (!float.IsNaN(pos.x) || !float.IsNaN(pos.y) || !float.IsNaN(pos.z))
            {
                transform.position = pos;
            }
            transform.rotation = UnityEngine.Quaternion.Euler(new UnityEngine.Vector3((float)((CollisionSphere)physicsObject).body.Rotation.x, (float)((CollisionSphere)physicsObject).body.Rotation.y, (float)((CollisionSphere)physicsObject).body.Rotation.z));

        }


    }

}
