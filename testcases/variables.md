# Testcase XML Specification Changes 

This document describes the changes made to the original SteerSuite with regards to the xml test case specification. 

See [the original documentation]() for the remaining information on the xml variable specification.

<!-- # `radius` 

Still referes to the physical dimensions of the person. People should never be within this distance from other items within the simulation, else they have collided. -->

# `sdradius`

Stands for social-distancing radius. Referes to the extra intereaction radius between other people. People will avoid objects using the `radius` value, but avoid other people with the `radius` + `sdradius` value. 

Should be set within the `initialConditions` tag.

If not set then will default to `0` and behave as if there is no social distancing.

# `seekStaticTargetSet`

An alternative goal type for use within the `goalSequence` tag. 

Consecutive sets of these goals will be considered together. The closest location of all the consecutive `seekStaticTargetSet` locations will be the chosen goal. Once the it reaches any goal specified by the sequential set of `seekStaticTargetSet` it will complete all the set and move onto the next goal in the queue.

An example use of this is when boarding a train. There are numerous doors to choose from, and as long as the agent reaches one of the goals (one of the doors) it does not care of the other locations at that point.

An example is provided below for 2 possible goals

```xml
    <goalSequence>
      <seekStaticTargetSet>
        <targetLocationsSet>
          <targetLocation>
            <x>-3</x>
            <y>0</y>
            <z>-2</z>
          </targetLocation>
          <targetLocation>
            <x>12</x>
            <y>0</y>
            <z>-2</z>
          </targetLocation>
        </targetLocationsSet>
        ...(goal info)...
      </seekStaticTargetSet>
      ...(other goals)...
    </goalSequence>
```

# goalSequence -> Behaviour -> Parameters

behaviour and parameter tags are already implemented in steersuite. Behaviour for some of these tags have been implemented, and are explained below.

A way to implement

``` xml
<goalSequence>
  <seekStaticTargetSet>
    <targetLocationsSet>
      ...
    </targetLocationsSet>
    ...
    <Behaviour>
      <Parameters>
        <lowPriority>
          <key>low priority</key>
          <value>1</value>
        </lowPriority>
        <boarding>
          <key>boarding</key>
          <value>1</value>
        </boarding>
      </Parameters>
    </Behaviour>
  </seekStaticTargetSet>
</goalSequence>

```

# `lowPriority`

A goal behaviour parameter. Active if set the key to `lowPriority` (value unimportant). If another nearby agent has the same goal, but does not have the `lowPriority` this agent will stand still (minus the steering) until no other agents without `lowPriority` are navigating to the same goal. 

Include these tags within a particular goal within the `goalSequence` tag.

# `boarding`

A goal behaviour parameter. Active if set the key to `lowPriority` (value unimportant). It will adjust the amount of avoidance between other nearby agents (regardless of the value of other agents). It does so by creating rules as if people are always facing the direction of the current goal. People will not avoid those they cannot see (e.g. behind them), while people behind others will take full responsibility since they know the people in front cannot see those behind.

Include these tags within a particular goal within the `goalSequence` tag.


# agent-> behaviour

Behaviour specification for agents. Comparing to the above use of `behaviour`, this tag is used for specifying optional behaviour that is goal independent. For example, certain environmental factors, like social distancing radius changing depending on the location of the agent, regardless of goal.

## sdradius_z

Specifies that the `sdradius` (social distancing radius) of the person should change as a function of the agent's `z` position. It is a linear change between `z0` and `z1` with values of `sd0` and `sd1` at those z values respectively. `z` values beyond this range will be clamped to the appropriate `sdx`. It should be specified as a child of the `agent` tag. Note the lack of capitalisation of `behaviour`.

An example agent tag will look like: 
```xml
<behaviour>
  <sdradius_z>
    <z0>-3</z0>
    <z1>0</z1>
    <sd0>0.2</sd0>
    <sd1>0</sd1>
  </sdradius_z>
</behaviour>
```