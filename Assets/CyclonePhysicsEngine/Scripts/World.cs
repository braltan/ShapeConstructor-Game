using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Cyclone
{
   public class World
    {
        bool calculateIterations;

        class BodyRegistration
        {
           public RigidBody body;
           public BodyRegistration next;
        }
        BodyRegistration firstBody;

        ContactResolver resolver;

        class ContactGenRegistration
        {
            public ContactGenerator gen;
            public ContactGenRegistration next;
        };

        ContactGenRegistration firstContactGen;

        Contact [] contacts;

        int maxContacts;

        public World(int maxContacts,int iterations = 0)
        {
            this.firstBody = null;
           //resolver = new ContactResolver(iterations);
            this.maxContacts = maxContacts;
            contacts = new Contact[maxContacts];
            calculateIterations = (iterations == 0);

        }
        public void startFrame()
        {
            BodyRegistration reg = firstBody;
            while(reg !=null)
            {
                reg.body.ClearAccumulators();
                reg.body.CalculateDerivedData();
                reg = reg.next;
            }
        }
        public int generateContacts()
        {
            int limit = maxContacts;
            Contact [] nextContact = contacts;

            ContactGenRegistration reg = firstContactGen;
            while (reg !=null)
            {
                int used = reg.gen.addContact(nextContact, limit);
                limit -= used;
              //  nextContact += used;

                if (limit <= 0) break;

                reg = reg.next;
            }
            return maxContacts - limit;
        }
        public void runPhysics(double duration)
        {
            // First apply the force generators
            //registry.updateForces(duration);

            // Then integrate the objects
            BodyRegistration  reg = firstBody;
            while (reg !=null)
            {
                // Remove all forces from the accumulator
                reg.body.Integrate(duration);

                // Get the next registration
                reg = reg.next;
            }

            // Generate contacts
            int usedContacts = generateContacts();

            // And process them
            if (calculateIterations) resolver.setIterations(usedContacts * 4);
            resolver.resolveContacts(contacts, usedContacts, duration);
        }
    };
}

