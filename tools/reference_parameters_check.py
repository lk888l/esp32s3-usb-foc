#!/usr/bin/env python3
"""Exercise v21/v22 independent reference parameters on an idle, calibrated, unloaded device.

Explicit enable/disable tests are bounded; no NVS writes. Finally restore the
initial RAM settings and leave the device idle. Use knobctl only as transport:
its implicit close/disable is deliberately not used for attachment checks.
"""
import argparse
import copy
import json
import time
from pathlib import Path
from knobctl import Knob


def configuration(snapshot, patches=None, mode=None):
    result = copy.deepcopy(snapshot['config'])
    result.pop('reference_modes', None)
    if patches is not None:
        result['reference_modes'] = patches
    if mode is not None:
        result['mode'] = mode
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM7')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    report = {'checks': [], 'requests': [], 'saved_nvs': False, 'passed': False}
    device = Knob(args.port)
    initial = None

    def request(op, config=None, **kwargs):
        result = device.request(op, config, **kwargs)
        report['requests'].append({'op': op, 'config': config, 'ok': result['ok'],
                                   'error': result.get('error'), 'state': result['state']})
        return result

    try:
        initial = request('status')
        assert initial['state'] == 'idle' and initial['fault'] == 'none', 'Requires initial idle / no fault'
        assert initial['reference_config_version'] in (1, 2) and initial['host_lease_ms'] == 0
        assert initial['calibration']['valid'] and initial['calibration_quality']['verified']
        report['initial'] = initial
        modern = initial['reference_config_version'] == 2
        bank = initial['config']['reference_modes']
        defaults = request('status', reference_defaults=True)
        assert defaults['config'] == initial['config']
        assert set(defaults['reference_defaults']) == set(map(str, range(4, 16)))
        report['checks'].append('default query is read-only and provides all 12 complete presets')

        for mode in range(4, 16):
            key = str(mode)
            updated = copy.deepcopy(bank[key])
            updated['strength'] += -.5 if updated['strength'] >= (1000 if modern else 100) else .5
            if modern:
                updated['click_enabled'] = not updated['click_enabled']
                updated['endstop_damping'] = .03125
            if mode == 14:
                updated['magnetic_positions'] = [3, 11, 23]
            result = request('configure', configuration(initial, {key: updated}))
            assert result['config']['reference_modes'][key] == updated
            assert all(result['config']['reference_modes'][other] == bank[other] for other in bank if other != key)
            assert result['calibration'] == initial['calibration']
            restored = request('configure', configuration(initial, {key: bank[key]}))
            assert restored['config'] == initial['config']
        report['checks'].append('12 independent editable presets apply/read back/restore without altering other modes or calibration')

        bad = []
        for field, value in [('width', 0), ('width', 4), ('strength', 1000.000001 if modern else 100.000001), ('damping', .501 if modern else 5.01),
                             ('scale', -1), ('endstop', 21), ('snap', .49), ('bias', .7),
                             ('friction', .451), ('click', .451), ('bounded', 1),
                             ('position', .5), ('position', 2), ('max_position', -1),
                             ('magnetic_positions', [0])]:
            candidate = copy.deepcopy(bank['8']); candidate[field] = value
            bad.append({ '8': candidate })
        if modern:
            for field, value in [('click_enabled', 1), ('endstop_damping', .501), ('endstop_damping', -1)]:
                candidate = copy.deepcopy(bank['8']); candidate[field] = value
                bad.append({'8': candidate})
            candidate = copy.deepcopy(bank['8']); candidate.pop('click_enabled')
            bad.append({'8': candidate})
        missing = copy.deepcopy(bank['8']); missing.pop('width')
        bad += [{'8': missing}, {'3': bank['8']}, {'16': bank['8']}]
        for magnets in ([2, 2], [32], list(range(9))):
            candidate = copy.deepcopy(bank['14']); candidate['magnetic_positions'] = magnets
            bad.append({'14': candidate})
        for patch in bad:
            rejected = request('configure', configuration(initial, patch), allow_error=True)
            assert not rejected['ok'], patch
            assert rejected['config'] == initial['config'] and rejected['state'] == 'idle'
        report['invalid_cases'] = len(bad)
        report['checks'].append('invalid bounds/types/records/magnets reject atomically without any configuration change')

        custom = copy.deepcopy(bank['4']); custom['strength'] = 1
        request('configure', configuration(initial, {'4': custom}))
        legacy = request('configure', configuration(initial, mode=2))
        assert legacy['config']['reference_modes']['4'] == custom
        request('configure', configuration(initial, {'4': bank['4']}))
        report['checks'].append('legacy configure omitting the bank preserves existing per-mode settings')

        changed = copy.deepcopy(bank['8']); changed['strength'] = 19
        changed['width'] = 3.141592653589793 / 4
        tuned = request('configure', configuration(initial, {'8': changed}, mode=8))
        request('enable')
        device.wait_active()
        active = request('status')
        assert active['state'] == 'active' and active['fault'] == 'none'
        forbidden = copy.deepcopy(active['config']['reference_modes']['8']); forbidden['strength'] = 20
        rejected = request('configure', configuration(active, {'8': forbidden}), allow_error=True)
        assert not rejected['ok'] and rejected['error'] == 'disable_before_reference_config'
        assert rejected['state'] == 'active' and rejected['config'] == active['config']
        noop = request('configure', configuration(active, {'8': active['config']['reference_modes']['8']}))
        assert noop['state'] == 'active' and noop['config'] == active['config']
        report['checks'].append('live parameter changes reject; identical entries are a no-op and preserve active state')
        samples = []
        for _ in range(20):
            time.sleep(.1)
            sample = request('status')
            assert sample['state'] == 'active' and sample['fault'] == 'none'
            assert sample['config'] == tuned['config']
            samples.append(sample)
        before = samples[-1]
        device.serial.close()
        time.sleep(5.5)
        device = Knob(args.port)
        after = request('status')
        assert after['state'] == 'active' and after['fault'] == 'none'
        assert after['config'] == before['config'] and after['calibration'] == before['calibration']
        assert after['uptime_ms'] - before['uptime_ms'] >= 5000
        assert after['control_cycles'] - before['control_cycles'] >= 24000
        report['detached'] = {'milliseconds': after['uptime_ms']-before['uptime_ms'],
                              'control_cycles': after['control_cycles']-before['control_cycles']}
        report['checks'].append('modified reference mode remains active independently for 5.5 seconds without USB traffic')
        report['passed'] = True
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        try:
            if initial is not None and initial['state'] == 'idle':
                request('disable')
                current = request('status')
                for key, original in initial['config']['reference_modes'].items():
                    if current['config']['reference_modes'][key] != original:
                        current = request('configure', configuration(initial, {key: original}))
                final = request('configure', configuration(initial))
                assert final['config'] == initial['config'] and final['state'] == 'idle' and final['fault'] == 'none'
                assert final['calibration'] == initial['calibration']
                assert final['calibration_quality'] == initial['calibration_quality']
                report['final'] = final
                report['restored'] = True
        except Exception as error:
            report['cleanup_error'] = str(error)
            report['passed'] = False
            raise
        finally:
            device.serial.close()
            args.out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'passed': report['passed'], 'checks': len(report['checks']),
                      'invalid_cases': report['invalid_cases'], 'detached': report['detached'], 'restored': report['restored']}))


if __name__ == '__main__':
    main()
