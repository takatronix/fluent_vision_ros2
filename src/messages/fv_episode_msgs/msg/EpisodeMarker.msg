# Episode marker event — recorded in rosbag as immutable audit log.
# canonical annotation is in meta.json / index.db (editable via PATCH).

string marker_id          # ULID
string episode_id         # ULID, parent episode
string action             # "start" | "stop" | "update"
string kind               # "subtask" | "event" | "note"
string task_description   # natural language label
string[] tags
string outcome            # action=stop only: "success" | "abort" | ""
int32 rev                 # canonical revision (initial publish = 0)
builtin_interfaces/Time stamp
