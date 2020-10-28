#!/usr/bin/env groovy

pipeline {
    agent any

    options {
        buildDiscarder(logRotator(numToKeepStr: '5', artifactNumToKeepStr: '3'))
    }

    stages {
        stage('Checkout') {
            steps {
                checkout([
                    $class: 'GitSCM',
                    branches: [
                        [name: "*/${env.BRANCH_NAME}"]
                    ],
                    browser: [
                        $class: 'GithubWeb',
                        repoUrl: 'https://github.com/PhischDotOrg/stm32f4-minimal'
                    ],
                    doGenerateSubmoduleConfigurations: false,
                    extensions: [
                        [ $class: 'SubmoduleOption',
                            depth: 1,
                            disableSubmodules: false,
                            parentCredentials: true,
                            recursiveSubmodules: true,
                            reference: '',
                            shallow: true,
                            threads: 8,
                            trackingSubmodules: false
                        ],
                        [ $class: 'CleanCheckout' ]
                    ],
                    submoduleCfg: [
                    ],
                    userRemoteConfigs: [
                        [
                            credentialsId: 'a88f9971-dae1-4a4d-9a8f-c7af88cad71b',
                            url: 'git@github.com:PhischDotOrg/stm32f4-minimal.git'
                        ]
                    ]
                ])
            }
        }
        stage('Build') {
            matrix {
                axes {
                    axis {
                        name 'STM32_ENVIRONMENT'
                        values 'Hostbuild',
                          'STM32'
                    }
                    axis {
                        name 'CMAKE_BUILD_TYPE'
                        values 'Debug',
                        //   'Release',
                          'ReleaseWithDebInfo',
                          'MinSizeRel'
                    }
                }
                excludes {
                    exclude {
                        axis {
                            name 'STM32_ENVIRONMENT'
                            values 'Hostbuild'
                        }
                        axis {
                            name 'CMAKE_BUILD_TYPE'
                            notValues 'Debug'
                        }
                    }
                }
                stages {
                    stage('Build') {
                        steps {
                            script {
                                if ("${STM32_ENVIRONMENT}" == "STM32") {
                                    stage ('CMake Build') {
                                        cmakeBuild buildDir: "build-${STM32_ENVIRONMENT}-${CMAKE_BUILD_TYPE}",
                                            buildType: "${CMAKE_BUILD_TYPE}",
                                            cleanBuild: true,
                                            cmakeArgs: "-DCMAKE_TOOLCHAIN_FILE=${WORKSPACE}/common/Generic_Cortex_M4.ctools",
                                            installation: 'InSearchPath',
                                            sourceDir: "${WORKSPACE}",
                                            steps: [
                                                [ args: 'all' ]
                                            ]
                                    }
                                } else if ("${STM32_ENVIRONMENT}" == "Hostbuild") {
                                    stage ('CMake Build') {
                                        cmakeBuild buildDir: "build-${STM32_ENVIRONMENT}-${CMAKE_BUILD_TYPE}",
                                            buildType: "${CMAKE_BUILD_TYPE}",
                                            cleanBuild: true,
                                            cmakeArgs: "-DUNITTEST=TRUE",
                                            installation: 'InSearchPath',
                                            sourceDir: "${WORKSPACE}",
                                            steps: [
                                                [ args: 'all' ]
                                            ]
                                    }
                                } else {
                                    error("STM_ENVIRONMENT='${STM32_ENVIRONMENT}' is not implemented")
                                }
                            }
                        }
                    }
                    stage('Test') {
                        steps {
                            ctest arguments: "-T test --no-compress-output",
                                installation: 'InSearchPath',
                                workingDir: "build-${STM32_ENVIRONMENT}-${CMAKE_BUILD_TYPE}"
                        }
                    }
                }

                post {
                    always {
                        // Archive the CTest xml output
                        archiveArtifacts (
                            artifacts: "build-${STM32_ENVIRONMENT}-${CMAKE_BUILD_TYPE}/Testing/**/*.xml",
                            fingerprint: true
                        )

                        // Process the CTest xml output with the xUnit plugin
                        xunit (
                            testTimeMargin: '3000',
                            thresholdMode: 1,
                            thresholds: [
                            skipped(failureThreshold: '0'),
                            failed(failureThreshold: '0')
                            ],
                        tools: [CTest(
                            pattern: "build-${STM32_ENVIRONMENT}-${CMAKE_BUILD_TYPE}/Testing/**/*.xml",
                            deleteOutputFiles: true,
                            failIfNotNew: false,
                            skipNoTestFiles: true,
                            stopProcessingIfError: true
                            )]
                        )
                    }
                }
            }
        }
    }
}
