# MiR RESET API

Need following python package:

```
pip install requests
```

## Information

Node name: _mir_api_server_

Service name: _/mir_api_

### Service calls
The service is a _int8_ and have following request and responses, if the request cant be made will all request respond with __0__.

| Request      | Number        | Response  |
| -------------|:-------------:| :---------:|
| Call MiR     | 1             |     1     |
| MiR arrived? | 2             |     2     |
| Release MiR  | 3             |     3     |
