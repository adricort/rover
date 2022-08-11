import yaml


while True:
    data = dict(
        A = 'a',
        B = dict(
            C = 'c',
            D = 'd',
            E = 'e',
        )
    )

    with open('data.yaml', 'w') as outfile:
        yaml.dump(data, outfile, default_flow_style=False)

