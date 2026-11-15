"""Bounded automatic approach using the validated single-step planner.

Defaults to a plan preview. --execute authorizes up to --max-steps movements.
Never retries a failed motion or reuses a stale vision target.
"""
import argparse
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true',help='execute successive planned steps')
    parser.add_argument('--max-steps',type=int,default=10,help='maximum movements (1-15)')
    args=parser.parse_args()
    if not 1<=args.max_steps<=15:
        parser.error('--max-steps must be 1..15')
    step=Path(__file__).with_name('approach_once.py')
    count=args.max_steps if args.execute else 1
    for index in range(count):
        print(f'Approach step {index+1}/{count}',flush=True)
        command=[sys.executable,str(step)]
        if args.execute:command.append('--execute')
        # Child owns action timeout/cancellation; never kill an in-flight motion on a wall timeout.
        completed=subprocess.run(command)
        if completed.returncode:
            print('Stopped: target, planning, feedback or execution check failed. No further steps.',flush=True)
            return completed.returncode
        if args.execute and index+1<count:
            time.sleep(2.)
    print('Completed bounded approach; stopped. No grasp command sent.',flush=True)
    return 0


if __name__=='__main__':
    try:sys.exit(main())
    except KeyboardInterrupt:
        print('Interrupted; no further approach steps.',flush=True)
        sys.exit(130)
