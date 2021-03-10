import subprocess


def hard_boot():
    output = subprocess.run('last -x --time-format iso -n 4 | tac', capture_output=True, shell=True)
    output = output.stdout.decode('utf-8')
    return 'shutdown' not in output


if __name__ == '__main__':
    print(hard_boot())
