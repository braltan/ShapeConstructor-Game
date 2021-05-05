using Cyclone.Math;

namespace Cyclone
{
    public class Gravity : IForceGenerator
    {
        private Vector3 gravity;

        public Gravity(Vector3 gravity)
        {
            this.gravity = gravity;
        }

        public void UpdateForce(RigidBody body, double duration)
        {
            if(body.HasFiniteMass() == false)
            {
                return;
            }

            body.AddForce(gravity * body.Mass);
        }
    }

    public class Spring : IForceGenerator
    {
        private Vector3 connectionPoint;

        private Vector3 otherConnectionPoint;

        private RigidBody other;

        private double springConstant;

        private double restLength;

        public Spring(Vector3 connectionPoint,
               RigidBody other,
               Vector3 otherConnectionPoint,
               double springConstant,
               double restLength)
        {
            this.connectionPoint = connectionPoint;
            this.otherConnectionPoint = otherConnectionPoint;
            this.other = other;
            this.springConstant = springConstant;
            this.restLength = restLength;
        }

        public virtual void UpdateForce(RigidBody body, double duration)
        {
            Vector3 lws = body.GetPointInWorldSpace(connectionPoint);
            Vector3 ows = other.GetPointInWorldSpace(otherConnectionPoint);

            Vector3 force = lws - ows;

            double magnitude = force.Magnitude;
            magnitude -= restLength;
            magnitude *= springConstant;

            force.Normalize();
            force *= -magnitude;
            body.AddForceAtPoint(force, lws);
        }
    };

    public class Aero : IForceGenerator
    {

        protected Matrix3 Tensor { get; set; }

        private Vector3 Position { get; set; }

        private Vector3 Windspeed { get; set; }

        public Aero(Matrix3 tensor, Vector3 position, Vector3 windspeed)
        {
            Tensor = tensor;
            Position = position;
            Windspeed = windspeed;
        }

        public virtual void UpdateForce(RigidBody body, double duration)
        {
            UpdateForceFromTensor(body, duration, Tensor);
        }

        protected void UpdateForceFromTensor(RigidBody body, double duration, Matrix3 tensor)
        {
            Vector3 velocity = body.GetVelocity();
            velocity += Windspeed;

            Vector3 bodyVel = body.GetTransform().TransformInverseDirection(velocity);

            Vector3 bodyForce = tensor.Transform(bodyVel);
            Vector3 force = body.GetTransform().TransformDirection(bodyForce);

            body.AddForceAtBodyPoint(force, Position);
        }
    }

    public class AeroControl : Aero
    {
        private Matrix3 maxTensor;

        private Matrix3 minTensor;

        private double controlSetting;

        private Matrix3 GetTensor()
        {
            if (controlSetting <= -1.0f)
            {
                return minTensor;
            }
            else if (controlSetting >= 1.0f)
            {
                return maxTensor;
            }
            else if (controlSetting < 0)
            {
                return Matrix3.LinearInterpolate(minTensor, Tensor, controlSetting + 1.0f);
            }
            else if (controlSetting > 0)
            {
                return Matrix3.LinearInterpolate(Tensor, maxTensor, controlSetting);
            }
            
            return Tensor;
        }

        public AeroControl(Matrix3 baseTensor, Matrix3 min, Matrix3 max, Vector3 position, Vector3 windspeed)
            : base(baseTensor, position, windspeed)
        {
            minTensor = min;
            maxTensor = max;
            controlSetting = 0.0f;
        }

        public void SetControl(double value)
        {
            controlSetting = value;
        }

        public override void UpdateForce(RigidBody body, double duration)
        {
            Matrix3 tensor = GetTensor();
            UpdateForceFromTensor(body, duration, tensor);
        }
    };
}
