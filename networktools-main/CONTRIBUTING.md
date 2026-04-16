# Contributing to networktools 


#### Table Of Contents


[Styleguides](#styleguides)
  * [Commit Convention](#commit-convention)
  * [C++ Styleguide](#cpp-styleguide)

## Styleguides

### Commit Convention

Commits are formatted as follows

`<type>(<scope>): <subject>`

where `<scope>` is optional

Example :

```
feat: add hat wobble
^--^  ^------------^
|     |
|     +-> Summary in present tense.
|
+-------> Type: chore, docs, feat, fix, refactor, style, or test.
```

Used types:

| Type     | description |
|:---------|:-------------|
| feat     | New feature for the user, not a new feature for build script |
| fix      | Bug fix for the user, not a fix to a build script |
| doc      | Changes to the documentation|
| style    | Formatting, missing semi colons, etc; no production code change|
| refactor | Refactoring production code, eg. renaming a variable|
| perf     | Performance improvements|
| test     | Adding missing tests, refactoring tests; no production code change|
| chore    | No production code change|

References:

- https://www.conventionalcommits.org/
- https://seesparkbox.com/foundry/semantic_commit_messages
- http://karma-runner.github.io/1.0/dev/git-commit-msg.html


### C++ Styleguide

Part of the naming conventions are taken/adapted from the LEMON library https://lemon.cs.elte.hu/trac/lemon/wiki/CodingStyle

#### Generals

* Reserve the use of `auto` for declaring objects of "complex" type. 
```c++
//-- Type returned by std::mem_fn is "complex" => use of auto ok
auto weightAccessor = std::mem_fn(p_weight_member);

//-- The type is rather "clear" => do not use auto
Arc arc = digraph.addArc(u, v);
```

#### Struct and Classes
* Prefer to declare a class with `struct` keyword to make data members public by default. Instead of declaring a member protected/private, prefixe it with a single underscore `_` to indicate that it should not be accessed outside the class. The reason for this choice is twofold:
  - It allows the library to be easily extendable/hackable without modifying it. A user may need access to a private data member to implement an ad-hoc feature that was unforseen by the library,
  - and yet, the prefix provides a hint to the user/client that a class member shouldn't be accessed/modified directly.

#### Naming

In order to make the development easier we have made some conventions
according to coding style. These include names of types, classes,
functions, variables, constants. If these conventions
are met in one's code then it is easier to read and maintain
it. Please comply with these conventions if you want to contribute
developing networktools library.

Note: When the coding style requires the capitalization of an abbreviation,
only the first letter should be upper case.

Warning: In some cases we diverge from these rules.
This is primary done because STL uses different naming convention and
in certain cases it is beneficial to provide STL compatible interface.

##### Structs
* The names of structures start with a capital letter and have a capital letter for each new word, with no underscores.
```c++
struct MyStruct {
  ...  
};
```

##### Variables and functions/methods

* The names of variables and data members are all lower case with underscores to separate each work. There is an additional prefix letter that indicates the type of a variable as follows
   * `i` for integer-like type (`int`, `unsigned`, `short`, ...)
   * `b` for boolean-like type (`bool`)
   * `f` for floating types (`float`, `double`, ..)
   * `sz` if the variable is a null-terminated string
   * `p` if the variable is a pointer

* The name of a function should look like the following.

```c++
firstWordLowerCaseRestCapitalizedWithoutUnderscores 
```

* Members of a classe have a prefixed underscore if it is protected or private. Public members are declared first, followed by protected and then private members.

```c++
struct MyStruct {

  char* sz_name;
  bool b_done;

  void fooBar() {}
  
  // Members that are primarily designed for internal use only
  int   _i_status;
  float _f_epsilon;
  
  void  _fooBarSecond() {}

};
```

##### Constants and template parameters
* The name of template arguments, constants and macros are all upper case with underscores. Template parameters are declared with the `class` keyword

```c++
#define EPSILON 0.01f

template <class TYPE>
struct MyStruct {
  ...
};
```

#### Header files

* Every header file should have #define guards to prevent multiple inclusion.

##### Template of a header

``` c++
#ifndef _NT_MY_STRUCT_H_
#define _NT_MY_STRUCT_H_

// Includes go here

namespace nt {
  namespace module {

    /**
     * @class MyStruct
     * @headerfile my_struct.h
     * @brief Brief description of what the struct does.
     * 
     */

    template < typename T_TYPE >
    struct MyStruct {
        // ...
    };
  }   // namespace module
}   // namespace nt

#endif
```



#### File Names

* Filenames should be all lowercase and can include underscores (_).
  * `my_struct.h` : contains the declaration of a struct MyStruct
