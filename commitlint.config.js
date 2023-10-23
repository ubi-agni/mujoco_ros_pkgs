// commitlint.config.js
module.exports = {
    extends: [
        '@commitlint/config-conventional' // scoped packages are not prefixed
    ],
	rules: {
		'type-enum': [
			2,
			'always',
			[
				'feat',
				'fix',
				'chg',
				'perf',
				'refactor',
				'docs',
				'test',
				'build',
				'revert',
				'gh',
			],
		],
	}
}
