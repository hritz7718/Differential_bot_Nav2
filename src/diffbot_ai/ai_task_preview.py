"""Preview Ollama task interpretations. Never sends commands to ROS."""

import argparse
import json
import urllib.error
import urllib.request

from task_protocol import TASK_SCHEMA, parse_task


def interpret(instruction, model, endpoint):
    payload = {
        'model': model,
        'stream': False,
        'format': TASK_SCHEMA,
        'options': {'temperature': 0},
        'messages': [
            {'role': 'system', 'content': (
                'Interpret the user request as an intent only. Do not invent target '
                'identities. Use reject for unsupported or ambiguous requests. '
                'Use stop when asked to stop. For approach_object use the requested '
                'object; default stop distance to 0.5 m and maximum speed to '
                '0.1 m/s. Include all fields even for stop or reject, using an '
                'empty target, 0.5 distance, and 0.1 speed. Output JSON matching: '
                + json.dumps(TASK_SCHEMA)
            )},
            {'role': 'user', 'content': instruction},
        ],
    }
    request = urllib.request.Request(
        endpoint,
        data=json.dumps(payload).encode('utf-8'),
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    with urllib.request.urlopen(request, timeout=90) as response:
        envelope = json.load(response)
    return parse_task(envelope['message']['content'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='qwen2.5:1.5b')
    parser.add_argument('--endpoint', default='http://127.0.0.1:11434/api/chat')
    args = parser.parse_args()
    print('Preview only: no ROS commands or navigation goals are sent.')
    while True:
        try:
            instruction = input('Instruction (quit to exit): ').strip()
            if instruction.lower() in ('quit', 'exit'):
                break
            if not instruction:
                continue
            print(json.dumps(interpret(instruction, args.model, args.endpoint), indent=2))
        except (EOFError, KeyboardInterrupt):
            print()
            break
        except (OSError, ValueError, KeyError, TypeError) as exc:
            print(f'Task rejected: {exc}')


if __name__ == '__main__':
    main()
