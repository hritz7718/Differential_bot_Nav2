"""Validation of untrusted, high-level AI requests. No motor control."""

import json
import math


TASK_SCHEMA = {
    'type': 'object',
    'properties': {
        'task': {'type': 'string', 'enum': ['approach_object', 'stop', 'reject']},
        'target': {'type': 'string'},
        'stop_distance_m': {'type': 'number'},
        'max_speed_mps': {'type': 'number'},
    },
    'required': ['task', 'target', 'stop_distance_m', 'max_speed_mps'],
    'additionalProperties': False,
}


def _no_duplicates(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate field: {key}')
        result[key] = value
    return result


def parse_task(raw):
    """Parse an LLM response, rejecting invalid or unsafe values.

    A valid request is an intent, not authorization to move.
    """
    task = json.loads(raw, object_pairs_hook=_no_duplicates)
    if not isinstance(task, dict) or set(task) != {
        'task', 'target', 'stop_distance_m', 'max_speed_mps'
    }:
        raise ValueError('Unexpected or missing task fields')
    if task['task'] not in ('approach_object', 'stop', 'reject'):
        raise ValueError('Unknown task')
    if not isinstance(task['target'], str) or len(task['target']) > 64:
        raise ValueError('Invalid target')
    target = task['target'].strip().lower()
    if task['task'] == 'approach_object' and not target:
        raise ValueError('An approach requires a target')
    for key, low, high in (('stop_distance_m', 0.5, 3.0), ('max_speed_mps', 0.01, 0.1)):
        value = task[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{key} must be a number')
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{key} must be between {low} and {high}')
    return {**task, 'target': target}
