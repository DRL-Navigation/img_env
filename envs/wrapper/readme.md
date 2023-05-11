### wrapper
Decorator Pattern (装饰器模式)
```
from env.wrapper import A,B,C

# Sequentially reading wrapper class and warper the env one by one.
env = ImageEnv()
env = A(env)
env = B(env)
env = C(env)
```


