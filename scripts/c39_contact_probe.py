"""Which contact route exists in THIS Isaac build? Run inside the sim container.

Measured on Isaac Sim 5.1 in this box (capture_dex5.json -> contact_api_hunt):

  isaacsim.core.prims.RigidContactView ......... ABSENT (module imports, attr missing)
  isaacsim.sensors.physics.ContactSensor ....... PRESENT
  omni.physx.get_physx_simulation_interface .... PRESENT
  articulation view .get_net_contact_forces .... ABSENT on both wrapper and physx view
  physx view .get_link_incoming_joint_force .... PRESENT  (JOINT REACTION, not contact)

So the two viable routes are a ContactSensor prim per body of interest, or the physx
simulation interface's contact-report callback. `get_link_incoming_joint_force` is NOT
a substitute: it is the joint reaction, and reading it as ground contact is the mistake
that put wrong foot forces in the C-39 write-up.

Run:  /isaac-sim/python.sh /scripts/c39_contact_probe.py
"""
print("--- route A: ContactSensor prim (preferred; per-body, gives force + count) ---")
try:
    from isaacsim.sensors.physics import ContactSensor
    print("  import OK. Attach one per body under test, e.g.:")
    print("    s = ContactSensor(prim_path='/World/G1/left_ankle_roll_link/contact',")
    print("                      translation=[0,0,0], radius=-1)")
    print("    s.initialize(); frame = s.get_current_frame()")
    print("    frame['number_of_contacts'], frame['force'], frame['contacts']")
    print("  NOTE: the prim must exist BEFORE the sim starts stepping.")
except Exception as exc:
    print(f"  unavailable: {exc!r}")

print("--- route B: physx contact-report callback (whole-scene, no per-body prims) ---")
try:
    from omni.physx import get_physx_simulation_interface
    import omni.physx.bindings._physx as pxb
    iface = get_physx_simulation_interface()
    print("  import OK. Subscribe once and decode in the callback:")
    print("    sub = iface.subscribe_contact_report_events(cb)")
    print("    # cb(contact_headers, contact_data): headers carry actor0/actor1,")
    print("    # data carries position, normal, impulse, separation per contact point.")
    print("  Requires PhysxContactReportAPI applied to the bodies you want reported.")
    print(f"  bindings: {[a for a in dir(pxb) if 'Contact' in a][:6]}")
except Exception as exc:
    print(f"  unavailable: {exc!r}")

print("--- route C: RigidContactView (NOT available in this build) ---")
try:
    import isaacsim.core.prims as P
    print(f"  RigidContactView present: {hasattr(P, 'RigidContactView')}")
except Exception as exc:
    print(f"  unavailable: {exc!r}")
