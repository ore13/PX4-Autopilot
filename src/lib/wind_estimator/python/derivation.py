import os
from symforce import symbolic as sm
from symforce import geo
from symforce import typing as T

def fuse_airspeed(
        v_local: geo.V3,
        state: geo.V3,
        P: geo.M33,
        airspeed: T.Scalar,
        R: T.Scalar,
        epsilon: T.Scalar
) -> geo.V3:

    vel_rel = geo.V3(v_local[0] - state[0], v_local[1] - state[1], v_local[2])
    airspeed_pred = vel_rel.norm(epsilon=epsilon) * state[2]

    innov = airspeed - airspeed_pred

    H = geo.V1(airspeed_pred).jacobian(state)
    innov_var = (H * P * H.transpose() + R)[0,0]

    K = P * H.transpose() / sm.Max(innov_var, epsilon)

    return (geo.V3(H), K, innov_var, innov)

from symforce.codegen import Codegen, CppConfig

codegen = Codegen.function(
        fuse_airspeed,
        output_names=["H", "K", "innov_var", "innov"],
        config=CppConfig())
metadata = codegen.generate_function(
        output_dir="generated",
        skip_directory_nesting=True)

print("Files generated in {}:\n".format(metadata.output_dir))
for f in metadata.generated_files:
    print("  |- {}".format(os.path.relpath(f, metadata.output_dir)))
