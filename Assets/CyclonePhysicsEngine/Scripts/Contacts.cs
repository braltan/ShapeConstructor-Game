
using Cyclone.Math;
using System;

namespace Cyclone
{
    public class ContactResolver
    {
        public int velocityIterations;
        public int positionIterations;
        public double velocityEpsilon;
        public double positionEpsilon;
        public int velocityIterationsUsed;
        public int positionIterationsUsed;
        public bool validSettings;

        public ContactResolver(int iterations, double velocityEpsilon = 0.01, double positionEpsilon = 0.01)
        {
            setIterations(iterations, iterations);
            setEpsilon(velocityEpsilon, positionEpsilon);
        }
        public ContactResolver(int velocityIterations,
                                 int positionIterations,
                                 double velocityEpsilon = 0.01,
                                 double positionEpsilon = 0.01)
        {
            setIterations(velocityIterations);
            setEpsilon(velocityEpsilon, positionEpsilon);
        }
       public void setIterations(int iterations)
        {
            setIterations(iterations, iterations);
        }
        void setIterations(int velocityIterations,
                                    int positionIterations)
        {
            this.velocityIterations = velocityIterations;
            this.positionIterations = positionIterations;
        }
        void setEpsilon(double velocityEpsilon,
                                 double positionEpsilon)
        {
            this.velocityEpsilon = velocityEpsilon;
            this.positionEpsilon = positionEpsilon;
        }
        public void resolveContacts(Contact [] contacts,
                                      int numContacts,
                                      double duration)
        {
            // Make sure we have something to do.
            if (numContacts == 0) return;
            if (!isValid()) return;

            // Prepare the contacts for processing
            prepareContacts(contacts, numContacts, duration);

            // Resolve the interpenetration problems with the contacts.
            adjustPositions(contacts, numContacts, duration);

            // Resolve the velocity problems with the contacts.
            adjustVelocities(contacts, numContacts, duration);
        }

        void prepareContacts(Contact [] contacts,
                                              int numContacts,
                                              double duration)
        {
            // Generate contact velocity and axis information.

            for (int i = 0 ; i < numContacts; i++)
            {
                contacts[i].calculateInternals(duration);
            }
        }

        void adjustVelocities(Contact [] c,
                                               int numContacts,
                                               double duration)
        {
            Vector3 [] velocityChange = new Vector3[2];
            velocityChange[0] = new Vector3();
            velocityChange[1] = new Vector3();
            Vector3 [] rotationChange = new Vector3[2];
            rotationChange[0] = new Vector3();
            rotationChange[1] = new Vector3();
            Vector3 deltaVel;

            // iteratively handle impacts in order of severity.
            velocityIterationsUsed = 0;
            while (velocityIterationsUsed < velocityIterations)
            {
                // Find contact with maximum magnitude of probable velocity change.
                double max = velocityEpsilon;
                int index = numContacts;
                for (int i = 0; i < numContacts; i++)
                {
                    if (c[i].desiredDeltaVelocity > max)
                    {
                        max = c[i].desiredDeltaVelocity;
                        index = i;
                    }
                }
                if (index == numContacts) break;

                // Match the awake state at the contact
                c[index].matchAwakeState();

                // Do the resolution on the contact that came out top.
                c[index].applyVelocityChange(velocityChange, rotationChange);

                // With the change in velocity of the two bodies, the update of
                // contact velocities means that some of the relative closing
                // velocities need recomputing.
                for (int i = 0; i < numContacts; i++)
                {
                    // Check each body in the contact
                    for (int b = 0; b < 2; b++) if (c[i].body[b] !=null)
                        {
                            // Check for a match with each body in the newly
                            // resolved contact
                            for (int d = 0; d < 2; d++)
                            {
                                if (c[i].body[b] == c[index].body[d])
                                {
                                    deltaVel = velocityChange[d] +
                                        rotationChange[d].VectorProduct(
                                            c[i].relativeContactPosition[b]);

                                    // The sign of the change is negative if we're dealing
                                    // with the second body in a contact.
                                    c[i].contactVelocity +=
                                        c[i].contactToWorld.TransformTranspose(deltaVel)
                                        * (b==1 ? -1 : 1);
                                    c[i].calculateDesiredDeltaVelocity(duration);
                                }
                            }
                        }
                }
                velocityIterationsUsed++;
            }
        }

        void adjustPositions(Contact[] c,
                                              int numContacts,
                                              double duration)
        {
            int i, index;
            Vector3[] linearChange = new Vector3[2];
            linearChange[0] = new Vector3();
            linearChange[1] = new Vector3();
            Vector3[] angularChange = new Vector3[2];
            angularChange[0] = new Vector3();
            angularChange[1] = new Vector3();
            double max;
            Vector3 deltaPosition;

            // iteratively resolve interpenetrations in order of severity.
            positionIterationsUsed = 0;
            while (positionIterationsUsed < positionIterations)
            {
                // Find biggest penetration
                max = positionEpsilon;
                index = numContacts;
                for (i = 0; i < numContacts; i++)
                {
                    if (c[i].penetration > max)
                    {
                        max = c[i].penetration;
                        index = i;
                    }
                }
                if (index == numContacts) break;

                // Match the awake state at the contact
                c[index].matchAwakeState();

                // Resolve the penetration.
                c[index].applyPositionChange(
                    linearChange,
                    angularChange,
                    max);

                // Again this action may have changed the penetration of other
                // bodies, so we update contacts.
                for (i = 0; i < numContacts; i++)
                {
                    // Check each body in the contact
                    for (int b = 0; b < 2; b++) if (c[i].body[b] != null)
                        {
                            // Check for a match with each body in the newly
                            // resolved contact
                            for (int d = 0; d < 2; d++)
                            {
                                if (c[i].body[b] == c[index].body[d])
                                {
                                    deltaPosition = linearChange[d] +
                                        angularChange[d].VectorProduct(
                                            c[i].relativeContactPosition[b]);

                                    // The sign of the change is positive if we're
                                    // dealing with the second body in a contact
                                    // and negative otherwise (because we're
                                    // subtracting the resolution)..
                                    c[i].penetration +=
                                        deltaPosition.ScalarProduct(c[i].contactNormal)
                                        * (b==1 ? 1 : -1);
                                }
                            }
                        }
                }
                positionIterationsUsed++;
            }
        }
            public bool isValid()
        {
            return (velocityIterations > 0) &&
                  (positionIterations > 0) &&
                  (positionEpsilon >= 0.0f) &&
                  (positionEpsilon >= 0.0f);
        }

    }
    public class ContactGenerator
    {
        public virtual int addContact(Contact[] contact, int limit)
        {
            return 0;
        }
    }
    public class Contact
    {
        public RigidBody[] body;
        public double friction;
        public double restitution;
        public Vector3 contactPoint;
        public Vector3 contactNormal;
        public double penetration;
        public Matrix3 contactToWorld;
        public Vector3 contactVelocity;
        public double desiredDeltaVelocity;
        public Vector3[] relativeContactPosition;

        public Contact()
        {
            body = new RigidBody[2];
            body[0] = new RigidBody();
            body[1] = new RigidBody();
            contactPoint = new Vector3();
            contactNormal = new Vector3();
            contactToWorld = new Matrix3();
            contactVelocity = new Vector3();
            relativeContactPosition = new Vector3[2];
            relativeContactPosition[0] = new Vector3();
            relativeContactPosition[1] = new Vector3();

        }
        public void setBodyData(RigidBody one, RigidBody two, double friction, double restitution)
        {
            body[0] = one;
            body[1] = two;
            this.friction = friction;
            this.restitution = restitution;
        }

        public void matchAwakeState()
        {
            if (body[1] == null)
                return;
            bool body0awake = body[0].GetAwake();
            bool body1awake = body[1].GetAwake();

            if (body0awake ^ body1awake)
            {
                if (body0awake)
                    body[1].SetAwake();
                else
                {
                    body[0].SetAwake();
                }
            }

        }
        public void swapBodies()
        {
            contactNormal *= -1;
            RigidBody temp = body[0];
            body[0] = body[1];
            body[1] = temp;
        }

        public void calculateContactBasis()
        {
            Vector3[] contactTangent = new Vector3[2];
            contactTangent[0] = new Vector3();
            contactTangent[1] = new Vector3();
            if (System.Math.Abs(contactNormal.x) > System.Math.Abs(contactNormal.y))
            {
                // Scaling factor to ensure the results are normalised
                double s = (double)1.0f / System.Math.Sqrt(contactNormal.z * contactNormal.z +
                   contactNormal.x * contactNormal.x);

                // The new X-axis is at right angles to the world Y-axis
                contactTangent[0].x = contactNormal.z * s;
                contactTangent[0].y = 0;
                contactTangent[0].z = -contactNormal.x * s;

                // The new Y-axis is at right angles to the new X- and Z- axes
                contactTangent[1].x = contactNormal.y * contactTangent[0].x;
                contactTangent[1].y = contactNormal.z * contactTangent[0].x -
                    contactNormal.x * contactTangent[0].z;
                contactTangent[1].z = -contactNormal.y * contactTangent[0].x;
            }
            else
            {
                // Scaling factor to ensure the results are normalised
                double s = (double)1.0 / System.Math.Sqrt(contactNormal.z * contactNormal.z +
                   contactNormal.y * contactNormal.y);

                // The new X-axis is at right angles to the world X-axis
                contactTangent[0].x = 0;
                contactTangent[0].y = -contactNormal.z * s;
                contactTangent[0].z = contactNormal.y * s;

                // The new Y-axis is at right angles to the new X- and Z- axes
                contactTangent[1].x = contactNormal.y * contactTangent[0].z -
                    contactNormal.z * contactTangent[0].y;
                contactTangent[1].y = -contactNormal.x * contactTangent[0].z;
                contactTangent[1].z = contactNormal.x * contactTangent[0].y;
            }

            // Make a matrix from the three vectors.
            contactToWorld = new Matrix3(
                contactNormal,
                contactTangent[0],
                contactTangent[1]);
        }
        public Vector3 calculateLocalVelocity(int bodyIndex, double duration)
        {
            RigidBody thisBody = body[bodyIndex];

            // Work out the velocity of the contact point.
            Vector3 velocity  = thisBody.GetRotation().VectorProduct(relativeContactPosition[bodyIndex]);

            velocity += thisBody.GetVelocity();

            // Turn the velocity into contact-coordinates.
            Vector3 contactVelocity = contactToWorld.TransformTranspose(velocity);

            // Calculate the ammount of velocity that is due to forces without
            // reactions.
            Vector3 accVelocity = thisBody.GetLastFrameAcceleration() * duration;

            // Calculate the velocity in contact-coordinates.
            accVelocity = contactToWorld.TransformTranspose(accVelocity);

            // We ignore any component of acceleration in the contact normal
            // direction, we are only interested in planar acceleration
            accVelocity.x = 0;

            // Add the planar velocities - if there's enough friction they will
            // be removed during velocity resolution
            contactVelocity += accVelocity;

            // And return it
            return contactVelocity;
        }
        public void calculateDesiredDeltaVelocity(double duration)
        {
            double velocityLimit = (double)0.25f;
            double velocityFromAcc = 0;

            if (body[0].GetAwake())
            {
                velocityFromAcc +=
                    body[0].GetLastFrameAcceleration() * duration * contactNormal;
            }

            if (body[1] != null && body[1].GetAwake())
            {
                velocityFromAcc -=
                    body[1].GetLastFrameAcceleration() * duration * contactNormal;
            }

            // If the velocity is very slow, limit the restitution
            double thisRestitution = restitution;
            if (System.Math.Abs(contactVelocity.x) < velocityLimit)
            {
                thisRestitution = (double)0.0f;
            }

            // Combine the bounce velocity with the removed
            // acceleration velocity.
            desiredDeltaVelocity =
                -contactVelocity.x
                - thisRestitution * (contactVelocity.x - velocityFromAcc);
        }

        public void calculateInternals(double duration)
        {
            // Check if the first object is NULL, and swap if it is.
            if (body[0] == null) swapBodies();
            if (Core.Equals(body[0], 0.0))
            {
                throw new Exception("mass cannot equal zero");
            }

            // Calculate an set of axis at the contact point.
            calculateContactBasis();

            // Store the relative position of the contact relative to each body
            relativeContactPosition[0] = contactPoint - body[0].GetPosition();
            if (body[1] != null)
            {
                relativeContactPosition[1] = contactPoint - body[1].GetPosition();
            }

            // Find the relative velocity of the bodies at the contact point.
            contactVelocity = calculateLocalVelocity(0, duration);
            if (body[1] != null)
            {
                contactVelocity -= calculateLocalVelocity(1, duration);
            }

            // Calculate the desired change in velocity for resolution
            calculateDesiredDeltaVelocity(duration);
        }
        public void applyVelocityChange(Vector3[] velocityChange, Vector3[] rotationChange)
        {
            // Get hold of the inverse mass and inverse inertia tensor, both in
            // world coordinates.
            Matrix3[] inverseInertiaTensor = new Matrix3[2];
            inverseInertiaTensor[0] = new Matrix3();
            inverseInertiaTensor[1] = new Matrix3();
            inverseInertiaTensor[0]= body[0].GetInverseInertiaTensorWorld(inverseInertiaTensor[0]);
            if (body[1] != null)
                inverseInertiaTensor[1] = body[1].GetInverseInertiaTensorWorld(inverseInertiaTensor[1]);

            // We will calculate the impulse for each contact axis
            Vector3 impulseContact;

            if (friction == (double)0.0)
            {
                // Use the short format for frictionless contacts
                impulseContact = calculateFrictionlessImpulse(inverseInertiaTensor);
            }
            else
            {
                // Otherwise we may have impulses that aren't in the direction of the
                // contact, so we need the more complex version.
                impulseContact = calculateFrictionImpulse(inverseInertiaTensor);
            }

            // Convert impulse to world coordinates
            Vector3 impulse = contactToWorld.Transform(impulseContact);

            // Split in the impulse into linear and rotational components

            Vector3 impulsiveTorque = relativeContactPosition[0].VectorProduct(impulse);
            rotationChange[0] = inverseInertiaTensor[0].Transform(impulsiveTorque);
            velocityChange[0].Clear();
            velocityChange[0].AddScaledVector(impulse, body[0].GetInverseMass());

            // Apply the changes
            body[0].AddVelocity(velocityChange[0]);
            body[0].AddRotation(rotationChange[0]);

            if (body[1] != null)
            {
                // Work out body one's linear and angular changes

                impulsiveTorque = impulse.VectorProduct(relativeContactPosition[1]);
                rotationChange[1] = inverseInertiaTensor[1].Transform(impulsiveTorque);
                velocityChange[1].Clear();
                velocityChange[1].AddScaledVector(impulse, body[1].GetInverseMass());

                // And apply them.
                body[1].AddVelocity(velocityChange[1]);
                body[1].AddRotation(rotationChange[1]);
            }
        }
        public Vector3 calculateFrictionlessImpulse(Matrix3 [] inverseInertiaTensor)
        {
            Vector3 impulseContact = new Vector3();

            // Build a vector that shows the change in velocity in
            // world space for a unit impulse in the direction of the contact
            // normal.
            Vector3 deltaVelWorld = relativeContactPosition[0].VectorProduct(contactNormal);
            deltaVelWorld = inverseInertiaTensor[0].Transform(deltaVelWorld);
            deltaVelWorld = deltaVelWorld.VectorProduct(relativeContactPosition[0]);

            // Work out the change in velocity in contact coordiantes.
            double deltaVelocity = deltaVelWorld * contactNormal;

            // Add the linear component of velocity change
            deltaVelocity += body[0].GetInverseMass();

            // Check if we need to the second body's data
            if (body[1] !=null)
            {
                // Go through the same transformation sequence again
                deltaVelWorld = relativeContactPosition[1].VectorProduct(contactNormal);
                deltaVelWorld = inverseInertiaTensor[1].Transform(deltaVelWorld);
                deltaVelWorld = deltaVelWorld.VectorProduct(relativeContactPosition[1]);

                // Add the change in velocity due to rotation
                deltaVelocity += deltaVelWorld * contactNormal;

                // Add the change in velocity due to linear motion
                deltaVelocity += body[1].GetInverseMass();
            }

            // Calculate the required size of the impulse
            impulseContact.x = desiredDeltaVelocity / deltaVelocity;
            impulseContact.y = 0;
            impulseContact.z = 0;
            return impulseContact;
        }
        public Vector3 calculateFrictionImpulse(Matrix3 [] inverseInertiaTensor)
        {
            Vector3 impulseContact = new Vector3();
            double inverseMass = body[0].GetInverseMass();

            // The equivalent of a cross product in matrices is multiplication
            // by a skew symmetric matrix - we build the matrix for converting
            // between linear and angular quantities.
            Matrix3 impulseToTorque = new Matrix3();
            impulseToTorque.SetSkewSymmetric(relativeContactPosition[0]);

            // Build the matrix to convert contact impulse to change in velocity
            // in world coordinates.
            Matrix3 deltaVelWorld = impulseToTorque;
            deltaVelWorld *= inverseInertiaTensor[0];
            deltaVelWorld *= impulseToTorque;
            deltaVelWorld *= -1;

            // Check if we need to add body two's data
            if (body[1] !=null)
            {
                // Set the cross product matrix
                impulseToTorque.SetSkewSymmetric(relativeContactPosition[1]);

                // Calculate the velocity change matrix
                Matrix3 deltaVelWorld2 = impulseToTorque;
                deltaVelWorld2 *= inverseInertiaTensor[1];
                deltaVelWorld2 *= impulseToTorque;
                deltaVelWorld2 *= -1;

                // Add to the total delta velocity.
                deltaVelWorld += deltaVelWorld2;

                // Add to the inverse mass
                inverseMass += body[1].GetInverseMass();
            }

            // Do a change of basis to convert into contact coordinates.
            Matrix3 deltaVelocity = contactToWorld.Transpose();
            deltaVelocity *= deltaVelWorld;
            deltaVelocity *= contactToWorld;

            // Add in the linear velocity change
            deltaVelocity.Data[0] += inverseMass;
            deltaVelocity.Data[4] += inverseMass;
            deltaVelocity.Data[8] += inverseMass;

            // Invert to get the impulse needed per unit velocity
            Matrix3 impulseMatrix = deltaVelocity.Inverse();

            // Find the target velocities to kill
            Vector3 velKill = new Vector3(desiredDeltaVelocity,
        -contactVelocity.y,
        -contactVelocity.z);

            // Find the impulse to kill target velocities
            impulseContact = impulseMatrix.Transform(velKill);

            // Check for exceeding friction
            double planarImpulse = System.Math.Sqrt(
                impulseContact.y * impulseContact.y +
                impulseContact.z * impulseContact.z
                );
            if (planarImpulse > impulseContact.x * friction)
            {
                // We need to use dynamic friction
                impulseContact.y /= planarImpulse;
                impulseContact.z /= planarImpulse;

                impulseContact.x = deltaVelocity.Data[0] +
                    deltaVelocity.Data[1] * friction * impulseContact.y +
                    deltaVelocity.Data[2] * friction * impulseContact.z;
                impulseContact.x = desiredDeltaVelocity / impulseContact.x;
                impulseContact.y *= friction * impulseContact.x;
                impulseContact.z *= friction * impulseContact.x;
            }
            return impulseContact;
        }
        public void applyPositionChange(Vector3 [] linearChange,
                                  Vector3 [] angularChange,
                                  double penetration)
        {
            const double angularLimit = (double)0.2f;
            double [] angularMove = new double [2];
            angularMove[0] = new double();
            angularMove[1] = new double();
            double [] linearMove= new double [2];
            linearMove[0] = new double();
            linearMove[1] = new double();

            double totalInertia = 0;
            double [] linearInertia= new double [2];
            linearInertia[0] = new double();
            linearInertia[1] = new double();
            double [] angularInertia= new double[2];
            angularInertia[0] = new double();
            angularInertia[1] = new double();

            // We need to work out the inertia of each object in the direction
            // of the contact normal, due to angular inertia only.
            for (int i = 0; i < 2; i++) if (body[i] !=null)
                {
                    Matrix3 inverseInertiaTensor = new Matrix3();
                    inverseInertiaTensor = body[i].GetInverseInertiaTensorWorld(inverseInertiaTensor);

                    // Use the same procedure as for calculating frictionless
                    // velocity change to work out the angular inertia.
                    Vector3 angularInertiaWorld =
                        relativeContactPosition[i].VectorProduct(contactNormal);
                    angularInertiaWorld =
                        inverseInertiaTensor.Transform(angularInertiaWorld);
                    angularInertiaWorld =
                        angularInertiaWorld .VectorProduct(relativeContactPosition[i]);
                    angularInertia[i] =
                        angularInertiaWorld * contactNormal;

                    // The linear component is simply the inverse mass
                    linearInertia[i] = body[i].GetInverseMass();

                    // Keep track of the total inertia from all components
                    totalInertia += linearInertia[i] + angularInertia[i];

                    // We break the loop here so that the totalInertia value is
                    // completely calculated (by both iterations) before
                    // continuing.
                }

            // Loop through again calculating and applying the changes
            for (int i = 0; i < 2; i++) if (body[i] !=null)
                {
                    // The linear and angular movements required are in proportion to
                    // the two inverse inertias.
                    double sign = (i == 0) ? 1 : -1;
                    angularMove[i] =
                        sign * penetration * (angularInertia[i] / totalInertia);
                    linearMove[i] =
                        sign * penetration * (linearInertia[i] / totalInertia);

                    // To avoid angular projections that are too great (when mass is large
                    // but inertia tensor is small) limit the angular move.
                    Vector3 projection = relativeContactPosition[i];
                    projection.AddScaledVector(
                        contactNormal,
                        -relativeContactPosition[i].ScalarProduct(contactNormal)
                        );

                    // Use the small angle approximation for the sine of the angle (i.e.
                    // the magnitude would be sine(angularLimit) * projection.magnitude
                    // but we approximate sine(angularLimit) to angularLimit).
                    double maxMagnitude = angularLimit * projection.Magnitude;

                    if (angularMove[i] < -maxMagnitude)
                    {
                        double totalMove = angularMove[i] + linearMove[i];
                        angularMove[i] = -maxMagnitude;
                        linearMove[i] = totalMove - angularMove[i];
                    }
                    else if (angularMove[i] > maxMagnitude)
                    {
                        double totalMove = angularMove[i] + linearMove[i];
                        angularMove[i] = maxMagnitude;
                        linearMove[i] = totalMove - angularMove[i];
                    }

                    // We have the linear amount of movement required by turning
                    // the rigid body (in angularMove[i]). We now need to
                    // calculate the desired rotation to achieve that.
                    if (angularMove[i] == 0)
                    {
                        // Easy case - no angular movement means no rotation.
                        angularChange[i].Clear();
                    }
                    else
                    {
                        // Work out the direction we'd like to rotate in.
                        Vector3 targetAngularDirection =
                            relativeContactPosition[i].VectorProduct(contactNormal);

                        Matrix3 inverseInertiaTensor = new Matrix3 ();
                        inverseInertiaTensor= body[i].GetInverseInertiaTensorWorld(inverseInertiaTensor);

                        // Work out the direction we'd need to rotate to achieve that
                        angularChange[i] =
                            inverseInertiaTensor.Transform(targetAngularDirection) *
                            (angularMove[i] / angularInertia[i]);
                    }

                    // Velocity change is easier - it is just the linear movement
                    // along the contact normal.
                    linearChange[i] = contactNormal * linearMove[i];

                    // Now we can start to apply the values we've calculated.
                    // Apply the linear movement
                    Vector3 pos = new Vector3 ();
                    body[i].GetPosition(pos);
                    pos.AddScaledVector(contactNormal, linearMove[i]);
                    body[i].SetPosition(pos);

                    // And the change in orientation
                    Quaternion q = new Quaternion();
                    body[i].GetOrientation(q);
                    q.AddScaledVector(angularChange[i], ((double)1.0));
                    body[i].SetOrientation(q);

                    // We need to calculate the derived data for any body that is
                    // asleep, so that the changes are reflected in the object's
                    // data. Otherwise the resolution will not change the position
                    // of the object, and the next collision detection round will
                    // have the same penetration.
                    if (!body[i].GetAwake()) body[i].CalculateDerivedData();
                }
        }


    }
}
