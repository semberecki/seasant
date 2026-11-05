"""The SesAnt family (sesant.*): SesAnt Jumper in Jumper's training framework.

SesAnt is a remix of RealAnt, Sesame, StackChan and Jumper (see the sesant repository). Its model
(`assets/sesant/sesant.xml`) is generated from the CAD by `sesant/cad/scripts/jumper_sim.py`; install this
package and the model with `sesant/sesant_jumper/sim/install_into_jumper.sh <jumper checkout>`.

Same layout as `tasks/jumper/`: `common/` holds the robot (not a task), each task directory holds its
register call, env_cfg.py and rl_cfg.py.
"""
